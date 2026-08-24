# asr_sdm_log_collector

Collects the log output of every process on the robot into one set of rotating
files, so debugging a run means reading one directory instead of scraping a
dozen terminals.

The package has two halves:

| Half | Target | Depends on ROS? | Role |
| --- | --- | --- | --- |
| Sending | `asr_sdm_log_client` (library) | No | Gives a process an spdlog logger that also ships every record to the collector. |
| Receiving | `asr_sdm_log_collector_node` | Yes | Listens for those datagrams, also subscribes to `/rosout`, and writes both into rotating files. |

Because the sending library has no ROS dependency, plain executables and
utilities can report into the same place as the nodes.

## Ingest paths

**Datagrams, for spdlog users.** A process links `asr_sdm_log_client`, calls
`initialize()`, and every record it logs goes to its console *and* to the
collector. Records are handed to a background thread and the socket is
non-blocking, so a control loop never waits on logging; if the queue fills, the
oldest record is dropped rather than stalling the caller.

Two transports carry those datagrams, and a sender picks exactly one, so records
are never duplicated:

| Transport | When it is used | Why |
| --- | --- | --- |
| Unix socket (`unix_socket.socket_path`) | Sender is on this host and the socket file already exists | Not silently lossy, cheaper than the network stack, and who may log is decided by file permissions |
| UDP (`udp.port`) | Otherwise | Reaches a collector on another machine |

The client prefers the socket and falls back to UDP, so a process that starts
before the collector still logs somewhere.

**`/rosout`, for everyone else.** Every node in this workspace currently logs
with `RCLCPP_*`, which lands on `/rosout`. The collector subscribes to it, so all
of those nodes are collected with no change to their code. New code can adopt the
spdlog client gradually.

All paths feed the same writer and the same files, so the merged log is a single
ordered view of the whole system.

### Moving the socket to a system path

The shipped defaults keep both the socket and the log files under
`~/log/vehicle`, so nothing needs root and a fresh checkout works as it is. The
socket file sits next to the per-run subdirectories rather than inside them.

A robot where the collector runs as a service usually wants a system path
instead. Those directories are not writable by an ordinary user, so create them
first — with a `tmpfiles.d` rule for `/run`, which is a tmpfs and is empty again
after every boot — and then point both halves at the socket:

```bash
ros2 launch asr_sdm_log_collector asr_sdm_log_collector.launch.py \
  socket_path:=/run/vehicle/log.sock log_directory:=/var/log/vehicle
ASR_SDM_LOG_COLLECTOR_SOCKET=/run/vehicle/log.sock \
  ros2 run asr_sdm_log_collector asr_sdm_log_client_demo asr_sdm_hardware 5
```

Note that a socket under `$HOME` is only reachable by accounts that can traverse
the home directory, so `unix_socket.permissions` only starts to matter once the
socket moves somewhere shared.

## Running the collector

```bash
colcon build --packages-select asr_sdm_log_collector
source install/setup.bash

ros2 launch asr_sdm_log_collector asr_sdm_log_collector.launch.py
# or, mirroring everything to the terminal while bringing a robot up:
ros2 launch asr_sdm_log_collector asr_sdm_log_collector.launch.py echo_to_console:=true
```

On startup it prints each transport it opened and the directory it chose. With
the shipped `sink.log_directory` of `~/log/vehicle`, output looks like:

```
~/log/vehicle/
├── log.sock                    # the Unix socket senders write to
├── latest -> 2026-08-24_08-45-12_31337
└── 2026-08-24_08-45-12_31337/
    ├── all.log                 # every node, in arrival order
    ├── asr_sdm_hardware.log
    ├── asr_sdm_teleop.log
    └── asr_sdm_log_collector.log
```

If `sink.log_directory` turns out not to be writable, the collector warns and
falls back to the ROS log location rather than refusing to start. Watch for this
line before going looking for the files:

```
[WARN] cannot use log directory '/var/log/vehicle': Permission denied.
       Falling back to /home/you/.ros/log/asr_sdm_log_collector/2026-08-24_08-45-12_31337
```

Each line keeps the timestamp, level, thread and source location of the process
that produced it, not of the collector:

```
[2026-08-24 08:45:13.117] [warning] [asr_sdm_hardware] [31402:31408] screw unit 3 stalled (uart2can.cpp:128)
```

## Sending from a node

Add the dependency:

```xml
<depend>asr_sdm_log_collector</depend>
```

```cmake
find_package(asr_sdm_log_collector REQUIRED)
target_link_libraries(my_node PRIVATE asr_sdm_log_collector::asr_sdm_log_client)

# Needed only if you want the TRACE and DEBUG macros to survive compilation.
target_compile_definitions(my_node PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
```

```cpp
#include <asr_sdm_log_collector/log_client.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  asr_sdm::log::initialize("asr_sdm_hardware");

  SPDLOG_INFO("joint {} at {:.3f} rad", index, position);
  SPDLOG_WARN("screw current {:.1f} A above nominal", current);

  rclcpp::spin(std::make_shared<MyNode>());

  asr_sdm::log::shutdown();   // flushes whatever is still queued
  rclcpp::shutdown();
  return 0;
}
```

Pass the ROS node name to `initialize()` so the collector's per-node file lines
up with the ROS graph. Use the `SPDLOG_*` macros rather than
`spdlog::info(...)`: only the macros capture file, line and function, and only
those reach the collector.

`initialize()` never throws. If the collector socket cannot be opened it logs a
single line to stderr and returns a console-only logger, because a logging
subsystem must not be able to take a node down.

### Environment overrides

Useful for pointing a process at a collector on another machine, or turning the
verbosity up on a robot that is already running, without editing launch files.

| Variable | Default | Meaning |
| --- | --- | --- |
| `ASR_SDM_LOG_COLLECTOR_SOCKET` | `~/log/vehicle/log.sock` | Unix socket to prefer, and it has to match the collector's `unix_socket.socket_path`. Accepts `~` and `$VAR`. Empty, or absent on disk, falls back to UDP. |
| `ASR_SDM_LOG_COLLECTOR_HOST` | `127.0.0.1` | Collector host name or IP, for the UDP fallback. |
| `ASR_SDM_LOG_COLLECTOR_PORT` | `9110` | Collector UDP port. |
| `ASR_SDM_LOG_LEVEL` | `info` | `trace`/`debug`/`info`/`warning`/`error`/`critical`. |
| `ASR_SDM_LOG_FLUSH_LEVEL` | `warning` | Level that forces an immediate flush. |
| `ASR_SDM_LOG_CONSOLE` | `1` | Also log to the process's own console. |
| `ASR_SDM_LOG_TO_COLLECTOR` | `1` | Ship records to the collector. |
| `ASR_SDM_LOG_QUEUE_SIZE` | `8192` | Background queue depth. |

## Verifying it works

```bash
ros2 launch asr_sdm_log_collector asr_sdm_log_collector.launch.py echo_to_console:=true &
ros2 run asr_sdm_log_collector asr_sdm_log_client_demo asr_sdm_hardware 5

tail -f ~/log/vehicle/latest/all.log
```

The demo emits every level so you can confirm the whole range arrives.

## Parameters

See `config/asr_sdm_log_collector.yaml`; every parameter is commented there. The
ones worth knowing about:

| Parameter | Default | Notes |
| --- | --- | --- |
| `minimum_level` | `trace` | Drops records below this level before any file is touched. |
| `unix_socket.socket_path` | `~/log/vehicle/log.sock` | Accepts `~` and `$VAR`. The parent directory is created when permissions allow. Senders must be pointed at the same path. |
| `unix_socket.permissions` | `"0666"` | Octal, as a string, since YAML's leading zero is not portable. Senders often run as another user. |
| `udp.bind_address` | `0.0.0.0` | Set to `127.0.0.1` to collect only from this host. |
| `udp.port` | `9110` | Must match the senders. |
| `rosout.enabled` | `true` | Set false to collect only spdlog senders. |
| `rosout.ignore_own_logs` | `true` | Keeps the collector's own status lines out of its intake. |
| `sink.log_directory` | `~/log/vehicle` | Accepts `~` and `$VAR`. Empty, or not writable, derives it from `ROS_LOG_DIR`, `ROS_HOME`, then `$HOME/.ros/log`. |
| `sink.log_filename` | `all.log` | The file that interleaves every node. |
| `sink.per_node_files` | `true` | One file per node next to the merged one. |
| `sink.max_file_size_mb` / `sink.max_files` | `100` / `10` | Rotation bounds, per file, so the total is multiplied by the number of files. |
| `sink.flush_level` | `warning` | Levels at or above this are flushed immediately. |
| `sink.flush_period_sec` | `2.0` | Bounds how much is lost if the collector is killed. |

`$VAR`, `${VAR}` and a leading `~` are expanded by the node itself, because a
parameter file is plain YAML with no shell involved; an unset variable expands to
nothing, as in a shell.

### Pattern flags

`sink.file_pattern` and `sink.console_pattern` take any
[spdlog pattern flag](https://github.com/gabime/spdlog/wiki/3.-Custom-formatting),
plus two added by this package because spdlog cannot express them for records
that arrived from another process:

| Flag | Meaning |
| --- | --- |
| `%*` | Process id of the *originating* process. Plain `%P` would give the collector's. |
| `%_` | ` (file:line)` of the originating call site, or nothing when unknown. |

`%n` is the originating node and `%t` its thread id. Records that arrived over
`/rosout` show `0:0` for `%*:%t`, because the ROS log message carries no process
or thread id; drop that part of the pattern if you find it noisy.

## Wire format

One record per datagram, on either transport, fields separated by ASCII Unit
Separator (`0x1f`), which never appears in log text:

```
ASR1 ␟ epoch_seconds.microseconds ␟ level ␟ logger ␟ pid ␟ tid ␟ file ␟ line ␟ function ␟ message
```

The message is last so it may contain separators and newlines of its own. It is
produced directly by an spdlog pattern (`asr_sdm::log::wire::pattern()`), so any
process that can set that pattern on a datagram sink can feed the collector
without linking this package.

Parsing never rejects a datagram: anything that does not carry the `ASR1` header
is stored as a plain message under the sender's fallback name, so a stray or
misconfigured sender shows up in the log rather than vanishing.

## Notes and limits

- **Sending never blocks, so it can drop.** The point is that logging can never
  stall or crash a control loop, which means a full send buffer costs a record.
  UDP can also lose one in the network. Records are counted, and
  `droppedRecordCount()` on the sending side plus the collector's periodic
  summary tell you if anything was lost.
- **Datagrams stay under 60000 bytes.** Longer records are truncated with a
  `...<truncated>` marker rather than dropped.
- **`sink.max_tracked_nodes` caps open files.** Nodes past the cap still land in
  the merged log, just without a file of their own.
- **The collector does not log its own ingest path.** Doing so would feed its
  own `/rosout` subscription, so problems are reported through the periodic
  summary instead.
