#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QProcess>
#include <QRegularExpression>
#include <QTimer>

#include <csignal>
#include <sys/types.h>

namespace
{
constexpr int kSimulatorStartTimeoutMs = 3000;
constexpr int kSimulatorSigintTimeoutMs = 5000;
constexpr int kSimulatorSigtermTimeoutMs = 2000;
constexpr int kSimulatorOutputLimit = 16000;

QString normalizeSimulatorOutput(QString text)
{
    static const QRegularExpression ansi(QStringLiteral("\\x1B\\[[0-9;]*[A-Za-z]"));
    text.remove(ansi);
    text.replace(QLatin1String("\r\n"), QLatin1String("\n"));
    text.replace(QLatin1Char('\r'), QLatin1Char('\n'));
    return text;
}

// ros2 launch and RViz are separate processes. Start them in their own
// process group so Stop can signal the whole tree without touching the monitor.
const char kProcessGroupPreamble[] =
    "import os, sys\n"
    "os.setpgrp()\n"
    "os.execvp(sys.argv[1], sys.argv[1:])\n";
}

QString RosUiBridge::planningSimulatorState() const
{
    return planning_simulator_state_;
}

QString RosUiBridge::planningSimulatorDetail() const
{
    return planning_simulator_detail_;
}

QString RosUiBridge::planningSimulatorLog() const
{
    return planning_simulator_output_;
}

bool RosUiBridge::isEnableDisableFlag(const QString &value)
{
    return value == QLatin1String("enable") || value == QLatin1String("disable");
}

void RosUiBridge::setPlanningSimulatorState(const QString &state, const QString &detail)
{
    if (planning_simulator_state_ == state && planning_simulator_detail_ == detail) {
        return;
    }
    planning_simulator_state_ = state;
    planning_simulator_detail_ = detail;
    if (!shutting_down_) {
        emit planningSimulatorChanged();
    }
}

void RosUiBridge::ensurePlanningSimulatorProcess()
{
    if (planning_simulator_process_) {
        return;
    }

    planning_simulator_process_ = new QProcess(this);
    planning_simulator_process_->setProcessChannelMode(QProcess::MergedChannels);
    connect(planning_simulator_process_, &QProcess::readyRead, this, [this]()
    {
        appendPlanningSimulatorOutput();
    });
    connect(planning_simulator_process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error)
    {
        if (error != QProcess::FailedToStart || shutting_down_) {
            return;
        }
        ++planning_simulator_stop_generation_;
        planning_simulator_stop_requested_ = false;
        setPlanningSimulatorState(QStringLiteral("failed"), QStringLiteral("start_failed"));
    });
    connect(planning_simulator_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int exitCode, QProcess::ExitStatus exitStatus)
    {
        onPlanningSimulatorFinished(exitCode, static_cast<int>(exitStatus));
    });
}

void RosUiBridge::appendPlanningSimulatorOutput()
{
    if (!planning_simulator_process_) {
        return;
    }
    planning_simulator_output_.append(
        normalizeSimulatorOutput(QString::fromLocal8Bit(planning_simulator_process_->readAll())));
    if (planning_simulator_output_.size() > kSimulatorOutputLimit) {
        planning_simulator_output_.remove(
            0, planning_simulator_output_.size() - kSimulatorOutputLimit);
    }
}

bool RosUiBridge::startPlanningSimulator(
    const QString &kind, const QString &control, const QString &teleop, const QString &planning)
{
    if (shutting_down_) {
        return false;
    }
    const QString package = kind == QLatin1String("logging")
        ? QStringLiteral("logging_simulator")
        : kind == QLatin1String("planning") ? QStringLiteral("planning_simulator") : QString();
    if (package.isEmpty()
        || !isEnableDisableFlag(control) || !isEnableDisableFlag(teleop) || !isEnableDisableFlag(planning)) {
        return false;
    }

    ensurePlanningSimulatorProcess();
    if (planning_simulator_process_->state() != QProcess::NotRunning) {
        return false;
    }

    planning_simulator_stop_requested_ = false;
    planning_simulator_output_.clear();
    ++planning_simulator_stop_generation_;
    setPlanningSimulatorState(QStringLiteral("running"), QString());

    const QStringList arguments = {
        QStringLiteral("-c"),
        QString::fromLatin1(kProcessGroupPreamble),
        QStringLiteral("ros2"),
        QStringLiteral("launch"),
        package,
        package + QStringLiteral(".launch.py"),
        QStringLiteral("control:=%1").arg(control),
        QStringLiteral("teleop:=%1").arg(teleop),
        QStringLiteral("planning:=%1").arg(planning),
    };
    planning_simulator_process_->start(QStringLiteral("python3"), arguments);
    if (!planning_simulator_process_->waitForStarted(kSimulatorStartTimeoutMs)) {
        if (planning_simulator_state_ == QLatin1String("running")) {
            setPlanningSimulatorState(QStringLiteral("failed"), QStringLiteral("start_failed"));
        }
        return false;
    }
    return planning_simulator_state_ == QLatin1String("running");
}

void RosUiBridge::stopPlanningSimulator()
{
    requestPlanningSimulatorStop(false);
}

void RosUiBridge::signalPlanningSimulator(int signalNumber) const
{
    if (!planning_simulator_process_) {
        return;
    }

    const qint64 pid = planning_simulator_process_->processId();
    if (pid > 0) {
        if (::kill(-static_cast<pid_t>(pid), signalNumber) != 0) {
            ::kill(static_cast<pid_t>(pid), signalNumber);
        }
        return;
    }

    if (signalNumber == SIGKILL) {
        planning_simulator_process_->kill();
    } else {
        planning_simulator_process_->terminate();
    }
}

void RosUiBridge::requestPlanningSimulatorStop(bool wait)
{
    if (!planning_simulator_process_ || planning_simulator_process_->state() == QProcess::NotRunning) {
        return;
    }

    const int generation = ++planning_simulator_stop_generation_;
    planning_simulator_stop_requested_ = true;
    if (!shutting_down_) {
        setPlanningSimulatorState(QStringLiteral("stopping"), QString());
    }

    const qint64 pid = planning_simulator_process_->processId();
    signalPlanningSimulator(SIGINT);

    if (wait) {
        if (!planning_simulator_process_->waitForFinished(kSimulatorSigintTimeoutMs)) {
            signalPlanningSimulator(SIGTERM);
            if (!planning_simulator_process_->waitForFinished(kSimulatorSigtermTimeoutMs)) {
                signalPlanningSimulator(SIGKILL);
                planning_simulator_process_->kill();
                planning_simulator_process_->waitForFinished(kSimulatorSigtermTimeoutMs);
            }
        }
        return;
    }

    QTimer::singleShot(kSimulatorSigintTimeoutMs, this, [this, generation, pid]()
    {
        if (generation != planning_simulator_stop_generation_ || pid <= 0) {
            return;
        }
        if (!planning_simulator_process_ || planning_simulator_process_->state() == QProcess::NotRunning) {
            return;
        }
        signalPlanningSimulator(SIGTERM);
    });
    QTimer::singleShot(kSimulatorSigintTimeoutMs + kSimulatorSigtermTimeoutMs, this, [this, generation, pid]()
    {
        if (generation != planning_simulator_stop_generation_ || pid <= 0) {
            return;
        }
        if (!planning_simulator_process_ || planning_simulator_process_->state() == QProcess::NotRunning) {
            return;
        }
        signalPlanningSimulator(SIGKILL);
        planning_simulator_process_->kill();
    });
}

void RosUiBridge::onPlanningSimulatorFinished(int exitCode, int exitStatus)
{
    ++planning_simulator_stop_generation_;
    appendPlanningSimulatorOutput();
    const bool requested = planning_simulator_stop_requested_;
    planning_simulator_stop_requested_ = false;
    if (shutting_down_) {
        return;
    }

    const bool cleanExit = exitStatus == static_cast<int>(QProcess::NormalExit) && exitCode == 0;
    if (requested || cleanExit) {
        setPlanningSimulatorState(QStringLiteral("idle"), QString());
        return;
    }

    QString detail = planning_simulator_output_.trimmed();
    if (detail.size() > 1500) {
        detail = detail.right(1500);
    }
    if (detail.isEmpty()) {
        detail = QStringLiteral("exit %1").arg(exitCode);
    }
    setPlanningSimulatorState(QStringLiteral("failed"), detail);
}
