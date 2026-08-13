#!/usr/bin/env python3

import argparse
import os
import re
import subprocess
import sys
try:
    import tomllib
except ModuleNotFoundError as exc:
    raise SystemExit(
        "install_pinocchio_from_source.py 需要 Python 3.11 或更高版本（标准库 tomllib）。"
    ) from exc
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

PINOCCHIO_REPO_URL = "https://github.com/stack-of-tasks/pinocchio.git"
EIGENPY_REPO_URL = "https://github.com/stack-of-tasks/eigenpy.git"
COAL_REPO_URL = "https://github.com/coal-library/coal.git"

INSTALL_PREFIX = Path(
    os.environ.get("PINOCCHIO_INSTALL_PREFIX", "/opt/openrobots")
).expanduser()
BUILD_COLLISION = os.environ.get("PINOCCHIO_BUILD_COLLISION", "OFF").upper() == "ON"
PYTHON_VERSION = f"{sys.version_info.major}.{sys.version_info.minor}"
BASHRC_PATH = Path.home() / ".bashrc"
STABLE_TAG_PATTERN = re.compile(r"^v?(\d+)\.(\d+)\.(\d+)$")
COMMIT_PATTERN = re.compile(r"^[0-9a-fA-F]{40}$")

APT_DEPENDENCIES = [
    "build-essential",
    "cmake",
    "git",
    "pkg-config",
    "python3-dev",
    "python3-numpy",
    "python3-pip",
    "python3-venv",
    "python3-pybind11",
    "libeigen3-dev",
    "libboost-all-dev",
    "liburdfdom-dev",
    "libassimp-dev",
    "libtinyxml2-dev",
]


@dataclass(frozen=True, order=True)
class Version:
    major: int
    minor: int = 0
    patch: int = 0

    @classmethod
    def parse(cls, value: str) -> "Version":
        match = re.fullmatch(r"v?(\d+)(?:\.(\d+))?(?:\.(\d+))?", value.strip())
        if not match:
            raise ValueError(f"不支持的版本格式: {value}")
        return cls(*(int(part or 0) for part in match.groups()))

    def __str__(self) -> str:
        return f"{self.major}.{self.minor}.{self.patch}"


@dataclass(frozen=True)
class SourceRef:
    name: str
    ref: str
    sha: str
    version: Version | None
    explicit: bool = False


@dataclass(frozen=True)
class StackSelection:
    pinocchio: SourceRef
    eigenpy: SourceRef
    coal: SourceRef
    eigenpy_constraints: tuple[str, ...]
    coal_constraints: tuple[str, ...]


@dataclass(frozen=True)
class Repository:
    name: str
    url: str
    src_dir: Path
    build_dir_override: Path | None


def run_cmd(cmd, check=True, capture_output=False, env=None, **kwargs):
    """执行命令并打印日志。"""
    print(f"执行命令: {' '.join(str(c) for c in cmd)}")
    return subprocess.run(
        cmd,
        check=check,
        capture_output=capture_output,
        text=capture_output,
        env=env,
        **kwargs,
    )


def command_output_with_env(cmd, env=None):
    """执行只读命令并返回标准输出，可显式指定环境。"""
    return run_cmd(cmd, capture_output=True, env=env).stdout.strip()


def command_output(cmd):
    """执行只读命令并返回标准输出。"""
    return command_output_with_env(cmd)


def ensure_dependencies():
    """安装 Pinocchio 源码构建所需的依赖。"""
    run_cmd(["sudo", "apt", "update"])
    run_cmd(["sudo", "apt", "install", "-qqy", *APT_DEPENDENCIES])


def stable_version(tag: str) -> Version | None:
    """将完整稳定 tag 转换为版本号，预发布和不完整 tag 返回 None。"""
    match = STABLE_TAG_PATTERN.fullmatch(tag)
    if not match:
        return None
    return Version(*(int(part) for part in match.groups()))


def parse_remote_tags(output: str, name: str) -> list[SourceRef]:
    """解析 ls-remote 输出，按语义版本从新到旧返回稳定 tag。"""
    refs: dict[str, dict[str, str]] = {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) != 2 or "refs/tags/" not in fields[1]:
            continue
        sha, full_ref = fields
        tag = full_ref.removeprefix("refs/tags/")
        peeled = tag.endswith("^{}")
        if peeled:
            tag = tag[:-3]
        version = stable_version(tag)
        if version is None:
            continue
        refs.setdefault(tag, {})["peeled" if peeled else "tag"] = sha

    releases = []
    for tag, shas in refs.items():
        releases.append(
            SourceRef(
                name=name,
                ref=tag,
                sha=shas.get("peeled", shas["tag"]),
                version=stable_version(tag),
            )
        )
    return sorted(releases, key=lambda release: release.version, reverse=True)


def list_stable_releases(name: str, repo_url: str) -> list[SourceRef]:
    """从远程仓库发现全部稳定语义版本 tag。"""
    output = command_output(["git", "ls-remote", "--tags", repo_url])
    releases = parse_remote_tags(output, name)
    if not releases:
        raise RuntimeError(f"{name} 仓库没有找到稳定语义版本 tag。")
    return releases


def github_raw_url(repo_url: str, ref: str, path: str) -> str:
    """为 canonical GitHub 仓库生成 raw 文件地址。"""
    match = re.fullmatch(r"https://github\.com/([^/]+)/([^/]+?)(?:\.git)?", repo_url)
    if not match:
        raise RuntimeError(f"无法为非 GitHub 仓库读取兼容元数据: {repo_url}")
    owner, repo = match.groups()
    return f"https://raw.githubusercontent.com/{owner}/{repo}/{ref}/{path}"


def fetch_pixi_toml(repo_url: str, ref: str) -> dict:
    """读取指定源码 ref 的 pixi.toml。"""
    url = github_raw_url(repo_url, ref, "pixi.toml")
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            return tomllib.loads(response.read().decode("utf-8"))
    except (
        urllib.error.URLError,
        TimeoutError,
        OSError,
        UnicodeDecodeError,
        tomllib.TOMLDecodeError,
    ) as exc:
        raise RuntimeError(f"无法读取兼容元数据 {url}: {exc}") from exc


def dependency_constraints(metadata: dict, dependency: str) -> tuple[str, ...]:
    """提取默认环境和 package host/run 表中的依赖约束。"""
    tables = [metadata.get("dependencies", {})]
    package = metadata.get("package", {})
    if isinstance(package, dict):
        tables.extend(
            [
                package.get("host-dependencies", {}),
                package.get("run-dependencies", {}),
            ]
        )

    constraints = []
    for table in tables:
        if not isinstance(table, dict):
            continue
        value = table.get(dependency)
        if isinstance(value, str):
            constraints.append(value.strip())
        elif isinstance(value, dict) and isinstance(value.get("version"), str):
            constraints.append(value["version"].strip())
    return tuple(dict.fromkeys(constraints))


def version_satisfies(version: Version, constraint: str) -> bool:
    """判断版本是否满足 Pixi/Conda 常用的比较、交集和 OR 约束。"""
    normalized = constraint.strip()
    if normalized in {"", "*"}:
        return True
    alternatives = [alternative.strip() for alternative in normalized.split("|")]
    if len(alternatives) > 1:
        return any(version_satisfies(version, alternative) for alternative in alternatives)

    for expression in normalized.split(","):
        expression = expression.strip()
        match = re.fullmatch(r"(>=|<=|==|=|>|<|~=)?\s*(v?\d+(?:\.\d+){0,2})(\.\*)?", expression)
        if not match:
            raise ValueError(f"不支持的版本约束: {constraint}")
        operator = match.group(1) or "=="
        required = Version.parse(match.group(2))
        wildcard = match.group(3) is not None

        if wildcard:
            specified = match.group(2).lstrip("v").count(".") + 1
            actual_parts = (version.major, version.minor, version.patch)
            required_parts = (required.major, required.minor, required.patch)
            if actual_parts[:specified] != required_parts[:specified]:
                return False
            continue

        comparisons = {
            ">=": version >= required,
            "<=": version <= required,
            ">": version > required,
            "<": version < required,
            "==": version == required,
            "=": version == required,
        }
        if operator == "~=":
            segment_count = match.group(2).lstrip("v").count(".") + 1
            if segment_count >= 3:
                upper = Version(required.major, required.minor + 1, 0)
            else:
                upper = Version(required.major + 1, 0, 0)
            if not (version >= required and version < upper):
                return False
        elif not comparisons[operator]:
            return False
    return True


def satisfies_all(version: Version | None, constraints: tuple[str, ...]) -> bool:
    """显式 branch/SHA 无版本号时留给 CMake 做最终验证。"""
    if version is None:
        return True
    return all(version_satisfies(version, constraint) for constraint in constraints)


def cached_metadata_loader(
    loader: Callable[[str, str], dict]
) -> Callable[[str, str], dict]:
    """缓存同一仓库/commit 的兼容元数据，避免回退搜索重复访问网络。"""
    cache: dict[tuple[str, str], dict | Exception] = {}

    def load(repo_url: str, ref: str) -> dict:
        key = (repo_url, ref)
        if key not in cache:
            try:
                cache[key] = loader(repo_url, ref)
            except Exception as exc:
                cache[key] = exc
        result = cache[key]
        if isinstance(result, Exception):
            raise result
        return result

    return load


def select_compatible_stack(
    pinocchio_candidates: list[SourceRef],
    eigenpy_candidates: list[SourceRef],
    coal_candidates: list[SourceRef],
    metadata_loader: Callable[[str, str], dict],
) -> StackSelection:
    """从候选版本中选择最新的完整兼容源码栈。"""
    failures = []
    metadata_loader = cached_metadata_loader(metadata_loader)
    for pinocchio in pinocchio_candidates:
        try:
            pinocchio_metadata = metadata_loader(PINOCCHIO_REPO_URL, pinocchio.sha)
            pin_eigenpy = dependency_constraints(pinocchio_metadata, "eigenpy")
            pin_coal = dependency_constraints(pinocchio_metadata, "coal")
            if not pin_eigenpy or not pin_coal:
                raise RuntimeError("pixi.toml 缺少 eigenpy 或 coal 约束")
        except (RuntimeError, ValueError) as exc:
            if pinocchio.explicit:
                print(f"警告: {pinocchio.ref} 无法完整解析兼容约束，将由 CMake 最终验证: {exc}")
                pin_eigenpy = ()
                pin_coal = ()
            else:
                failures.append(f"Pinocchio {pinocchio.ref}: {exc}")
                continue

        for coal in coal_candidates:
            try:
                if not satisfies_all(coal.version, pin_coal):
                    continue
                coal_metadata = metadata_loader(COAL_REPO_URL, coal.sha)
                coal_eigenpy = dependency_constraints(coal_metadata, "eigenpy")
                if not coal_eigenpy:
                    raise RuntimeError("pixi.toml 缺少 eigenpy 约束")
            except (RuntimeError, ValueError) as exc:
                if coal.explicit:
                    print(f"警告: {coal.ref} 无法完整解析 eigenpy 约束，将由 CMake 最终验证: {exc}")
                    coal_eigenpy = ()
                else:
                    failures.append(f"Coal {coal.ref}: {exc}")
                    continue

            eigenpy_constraints = tuple(dict.fromkeys((*pin_eigenpy, *coal_eigenpy)))
            for eigenpy in eigenpy_candidates:
                try:
                    if satisfies_all(eigenpy.version, eigenpy_constraints):
                        return StackSelection(
                            pinocchio=pinocchio,
                            eigenpy=eigenpy,
                            coal=coal,
                            eigenpy_constraints=eigenpy_constraints,
                            coal_constraints=pin_coal,
                        )
                except ValueError as exc:
                    failures.append(f"eigenpy {eigenpy.ref}: {exc}")
                    break

    detail = "\n  - ".join(failures[-10:])
    raise RuntimeError(f"没有找到可解析的稳定兼容版本组合。\n  - {detail}")


def find_override(new_name: str, legacy_name: str | None = None) -> str | None:
    """读取 ref 覆盖，并兼容旧的 *_BRANCH 环境变量。"""
    value = os.environ.get(new_name)
    if value:
        return value
    if legacy_name and os.environ.get(legacy_name):
        print(f"警告: {legacy_name} 已弃用，请改用 {new_name}。")
        return os.environ[legacy_name]
    return None


def resolve_override(
    name: str, repo_url: str, override: str, releases: list[SourceRef]
) -> SourceRef:
    """把显式 tag、branch 或 SHA 覆盖解析为不可变提交。"""
    for release in releases:
        if override == release.ref:
            return SourceRef(name, release.ref, release.sha, release.version, explicit=True)

    override_version = stable_version(override)
    version_matches = [
        release
        for release in releases
        if override_version is not None and release.version == override_version
    ]
    if len(version_matches) == 1:
        release = version_matches[0]
        return SourceRef(name, release.ref, release.sha, release.version, explicit=True)

    tag_output = command_output(
        ["git", "ls-remote", "--tags", repo_url, f"refs/tags/{override}*"]
    )
    if tag_output:
        tag_lines = tag_output.splitlines()
        exact_sha = None
        peeled_sha = None
        for line in tag_lines:
            sha, full_ref = line.split()[:2]
            if full_ref == f"refs/tags/{override}":
                exact_sha = sha
            elif full_ref == f"refs/tags/{override}^{{}}":
                peeled_sha = sha
        if exact_sha:
            version = stable_version(override)
            return SourceRef(
                name,
                override,
                peeled_sha or exact_sha,
                version,
                explicit=True,
            )

    branch_output = command_output(
        ["git", "ls-remote", "--heads", repo_url, f"refs/heads/{override}"]
    )
    if branch_output:
        sha = branch_output.split()[0]
        return SourceRef(name, override, sha, None, explicit=True)

    if COMMIT_PATTERN.fullmatch(override):
        print(f"警告: 使用显式 commit {override}，其可达性将在 checkout 时验证。")
        return SourceRef(name, override, override, None, explicit=True)
    raise RuntimeError(f"{name} 覆盖 ref 不存在: {override}")


def candidates_with_override(
    name: str, repo_url: str, releases: list[SourceRef], override: str | None
) -> list[SourceRef]:
    if not override:
        return releases
    return [resolve_override(name, repo_url, override, releases)]


def release_candidates(
    name: str, repo_url: str, override: str | None
) -> list[SourceRef]:
    """仅在自动模式枚举稳定 tag；显式 ref 可直接解析。"""
    if override:
        return [resolve_override(name, repo_url, override, [])]
    return list_stable_releases(name, repo_url)


def resolve_stack() -> StackSelection:
    """访问上游仓库并解析本次应安装的版本组合。"""
    pinocchio_override = find_override("PINOCCHIO_REF", "PINOCCHIO_BRANCH")
    eigenpy_override = find_override("EIGENPY_REF", "EIGENPY_BRANCH")
    coal_override = find_override("COAL_REF")

    pinocchio_releases = release_candidates(
        "Pinocchio", PINOCCHIO_REPO_URL, pinocchio_override
    )
    eigenpy_releases = release_candidates("eigenpy", EIGENPY_REPO_URL, eigenpy_override)
    coal_releases = release_candidates("Coal", COAL_REPO_URL, coal_override)

    return select_compatible_stack(
        pinocchio_releases,
        eigenpy_releases,
        coal_releases,
        fetch_pixi_toml,
    )


def print_selection(selection: StackSelection):
    """在修改系统前显示可审计的版本解析结果。"""
    print("\n自动解析出的源码版本组合:")
    for component in (selection.pinocchio, selection.eigenpy, selection.coal):
        version = str(component.version) if component.version else "显式非 tag ref"
        print(f"  {component.name:<10} {component.ref:<16} {component.sha} ({version})")
    print(f"  eigenpy 约束: {', '.join(selection.eigenpy_constraints) or '由 CMake 验证'}")
    print(f"  Coal 约束:    {', '.join(selection.coal_constraints) or '由 CMake 验证'}\n")


def normalize_repo_url(url: str) -> str:
    """将常见 GitHub HTTPS/SSH URL 归一化为 github.com/owner/repo。"""
    stripped = url.strip().removesuffix(".git").rstrip("/")
    patterns = (
        r"https?://github\.com/(.+)",
        r"ssh://git@github\.com/(.+)",
        r"git@github\.com:(.+)",
    )
    for pattern in patterns:
        match = re.fullmatch(pattern, stripped)
        if match:
            return f"github.com/{match.group(1).lower()}"
    return stripped


def repository_is_clean(src_dir: Path) -> bool:
    """检查顶层工作树和全部递归 submodule，忽略本地 ignore 配置。"""
    dirty = command_output(
        [
            "git",
            "-C",
            str(src_dir),
            "status",
            "--porcelain",
            "--ignore-submodules=none",
            "--untracked-files=all",
        ]
    )
    if dirty:
        return False
    submodule_env = os.environ.copy()
    submodule_env["LC_ALL"] = "C"
    submodule_status = command_output_with_env(
        [
            "git",
            "-C",
            str(src_dir),
            "submodule",
            "foreach",
            "--quiet",
            "--recursive",
            (
                'test -z "$(git status --porcelain --untracked-files=all)" '
                '|| printf "%s\\n" "$sm_path"'
            ),
        ],
        env=submodule_env,
    )
    return not submodule_status.strip()


def update_submodules(src_dir: Path):
    """同步 URL 并更新仓库子模块。"""
    run_cmd(["git", "-C", str(src_dir), "submodule", "sync", "--recursive"])
    run_cmd(
        [
            "git",
            "-C",
            str(src_dir),
            "submodule",
            "update",
            "--init",
            "--recursive",
            "--depth",
            "1",
        ]
    )


def preflight_repository(repository: Repository):
    """在任何 checkout 前验证已有源码目录的来源和干净状态。"""
    src_dir = repository.src_dir
    if not src_dir.exists():
        return
    if not (src_dir / ".git").exists():
        raise RuntimeError(f"{src_dir} 已存在但不是 Git 仓库。")
    origin = command_output(["git", "-C", str(src_dir), "remote", "get-url", "origin"])
    if normalize_repo_url(origin) != normalize_repo_url(repository.url):
        raise RuntimeError(f"{src_dir} 的 origin 不是预期仓库: {origin}")
    if not repository_is_clean(src_dir):
        raise RuntimeError(f"{src_dir} 或其 submodule 存在未提交修改，拒绝覆盖。")


def checkout_repository(repository: Repository, source_ref: SourceRef):
    """安全地将源码目录切换到已解析的不可变提交。"""
    src_dir = repository.src_dir
    preflight_repository(repository)
    if src_dir.exists():
        run_cmd(["git", "-C", str(src_dir), "fetch", "origin", "--tags", "--prune"])
    else:
        src_dir.parent.mkdir(parents=True, exist_ok=True)
        run_cmd(["git", "clone", "--no-checkout", repository.url, str(src_dir)])

    if COMMIT_PATTERN.fullmatch(source_ref.ref) and source_ref.ref == source_ref.sha:
        run_cmd(["git", "-C", str(src_dir), "fetch", "origin", source_ref.sha])
    run_cmd(["git", "-C", str(src_dir), "checkout", "--detach", source_ref.sha])
    actual_sha = command_output(["git", "-C", str(src_dir), "rev-parse", "HEAD"])
    if not actual_sha.startswith(source_ref.sha) and not source_ref.sha.startswith(actual_sha):
        raise RuntimeError(
            f"{repository.name} checkout 结果 {actual_sha} 与解析提交 {source_ref.sha} 不一致。"
        )
    update_submodules(src_dir)
    if not repository_is_clean(src_dir):
        raise RuntimeError(f"{src_dir} 的 submodule 更新后仍包含本地修改。")


def effective_cpu_count() -> int:
    """返回当前进程实际可用的逻辑 CPU 数，考虑 affinity/cgroup 限制。"""
    process_cpu_count = getattr(os, "process_cpu_count", None)
    if process_cpu_count:
        detected = process_cpu_count()
        if detected:
            return detected
    if hasattr(os, "sched_getaffinity"):
        try:
            affinity_count = len(os.sched_getaffinity(0))
            if affinity_count:
                return affinity_count
        except OSError:
            pass
    return os.cpu_count() or 1


def build_jobs(cpu_count: int | None = None) -> int:
    """使用当前进程可用逻辑 CPU 的一半，且至少使用一个 job。"""
    detected = effective_cpu_count() if cpu_count is None else cpu_count
    return max(1, (detected or 1) // 2)


def build_directory(repository: Repository, source_ref: SourceRef) -> Path:
    if repository.build_dir_override:
        return repository.build_dir_override
    safe_ref = re.sub(r"[^A-Za-z0-9_.-]+", "-", source_ref.ref)
    return repository.src_dir / f"build-{safe_ref}-{source_ref.sha[:8]}"


def cmake_environment() -> dict[str, str]:
    env = os.environ.copy()
    existing = env.get("CMAKE_PREFIX_PATH", "")
    env["CMAKE_PREFIX_PATH"] = (
        f"{INSTALL_PREFIX}{os.pathsep}{existing}" if existing else str(INSTALL_PREFIX)
    )
    return env


def install_command(build_dir: Path) -> list[str]:
    """用户可写前缀直接安装，受保护前缀才使用 sudo。"""
    probe = INSTALL_PREFIX
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    command = ["cmake", "--install", str(build_dir)]
    if not os.access(probe, os.W_OK):
        command.insert(0, "sudo")
    return command


def configure_build_install(
    repository: Repository, source_ref: SourceRef, cmake_options: list[str], jobs: int
):
    """配置、构建并安装一个源码组件。"""
    build_dir = build_directory(repository, source_ref)
    build_dir.mkdir(parents=True, exist_ok=True)
    common_options = [
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_INSTALL_PREFIX={INSTALL_PREFIX}",
        "-DBUILD_TESTING=OFF",
        "-DINSTALL_DOCUMENTATION=OFF",
        f"-DPYTHON_EXECUTABLE={sys.executable}",
    ]
    env = cmake_environment()
    run_cmd(
        [
            "cmake",
            "-S",
            str(repository.src_dir),
            "-B",
            str(build_dir),
            *common_options,
            *cmake_options,
        ],
        env=env,
    )
    run_cmd(
        ["cmake", "--build", str(build_dir), "--config", "Release", "-j", str(jobs)],
        env=env,
    )
    run_cmd(install_command(build_dir), env=env)


def repository_from_environment(
    name: str, url: str, src_variable: str, default_dir: str, build_variable: str
) -> Repository:
    build_override = os.environ.get(build_variable)
    return Repository(
        name=name,
        url=url,
        src_dir=Path(
            os.environ.get(src_variable, str(Path.home() / "src" / default_dir))
        ).expanduser(),
        build_dir_override=Path(build_override).expanduser() if build_override else None,
    )


def repositories() -> dict[str, Repository]:
    return {
        "pinocchio": repository_from_environment(
            "Pinocchio",
            PINOCCHIO_REPO_URL,
            "PINOCCHIO_SRC_DIR",
            "pinocchio",
            "PINOCCHIO_BUILD_DIR",
        ),
        "eigenpy": repository_from_environment(
            "eigenpy", EIGENPY_REPO_URL, "EIGENPY_SRC_DIR", "eigenpy", "EIGENPY_BUILD_DIR"
        ),
        "coal": repository_from_environment(
            "Coal", COAL_REPO_URL, "COAL_SRC_DIR", "coal", "COAL_BUILD_DIR"
        ),
    }


def coal_cmake_options(source_ref: SourceRef) -> list[str]:
    """按 Coal major 选择有效的可选依赖开关。"""
    options = [
        "-DBUILD_PYTHON_INTERFACE=ON",
        "-DBUILD_STANDALONE_PYTHON_INTERFACE=OFF",
        "-DGENERATE_PYTHON_STUBS=OFF",
    ]
    if source_ref.version is None or source_ref.version.major >= 3:
        options.extend(["-DCOAL_HAS_QHULL=OFF", "-DCOAL_ENABLE_LOGGING=OFF"])
    else:
        options.append("-DHPP_FCL_HAS_QHULL=OFF")
    return options


def install_stack(selection: StackSelection, jobs: int):
    """按 eigenpy、Coal、Pinocchio 顺序构建完整源码栈。"""
    repos = repositories()
    ordered = [
        ("eigenpy", selection.eigenpy),
        ("coal", selection.coal),
        ("pinocchio", selection.pinocchio),
    ]
    for key, _ in ordered:
        preflight_repository(repos[key])
    for key, source_ref in ordered:
        checkout_repository(repos[key], source_ref)

    configure_build_install(
        repos["eigenpy"],
        selection.eigenpy,
        ["-DBUILD_TESTING_SCIPY=OFF", "-DGENERATE_PYTHON_STUBS=OFF"],
        jobs,
    )
    configure_build_install(
        repos["coal"],
        selection.coal,
        coal_cmake_options(selection.coal),
        jobs,
    )
    configure_build_install(
        repos["pinocchio"],
        selection.pinocchio,
        [
            "-DBUILD_EXAMPLES=OFF",
            "-DBUILD_PYTHON_INTERFACE=ON",
            f"-DBUILD_WITH_COLLISION_SUPPORT={'ON' if BUILD_COLLISION else 'OFF'}",
        ],
        jobs,
    )


def update_bashrc():
    """添加或更新 ~/.bashrc 中由本脚本管理的环境变量。"""
    start_marker = "# >>> pinocchio source install setup >>>"
    end_marker = "# <<< pinocchio source install setup <<<"
    env_block = f"""{start_marker}
export PATH={INSTALL_PREFIX}/bin:$PATH
export PKG_CONFIG_PATH={INSTALL_PREFIX}/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH={INSTALL_PREFIX}/lib:$LD_LIBRARY_PATH
export PYTHONPATH={INSTALL_PREFIX}/lib/python{PYTHON_VERSION}/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH={INSTALL_PREFIX}:$CMAKE_PREFIX_PATH
{end_marker}"""

    existing = BASHRC_PATH.read_text(encoding="utf-8") if BASHRC_PATH.exists() else ""
    pattern = re.compile(
        rf"{re.escape(start_marker)}.*?{re.escape(end_marker)}", re.DOTALL
    )
    if pattern.search(existing):
        updated = pattern.sub(env_block, existing)
    else:
        separator = "" if not existing or existing.endswith("\n") else "\n"
        updated = f"{existing}{separator}\n{env_block}\n"
    BASHRC_PATH.write_text(updated, encoding="utf-8")
    print("已将 Pinocchio 环境变量写入 ~/.bashrc，请重新加载或重新登录终端。")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="自动选择并源码安装最新稳定兼容的 Pinocchio/eigenpy/Coal。"
    )
    parser.add_argument(
        "--resolve-only",
        action="store_true",
        help="只解析并显示兼容版本，不安装依赖、不修改源码目录。",
    )
    parser.add_argument(
        "--skip-dependencies",
        action="store_true",
        help="跳过 apt 构建依赖安装。",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    selection = resolve_stack()
    print_selection(selection)
    if args.resolve_only:
        return

    cpu_count = effective_cpu_count()
    jobs = build_jobs(cpu_count)
    print(f"检测到 {cpu_count} 个逻辑 CPU，三个项目均使用 {jobs} 个并行构建 job。")
    if not args.skip_dependencies:
        ensure_dependencies()
    install_stack(selection, jobs)
    update_bashrc()
    print("Pinocchio、eigenpy 和 Coal 源码安装及环境配置完成。")


if __name__ == "__main__":
    main()
