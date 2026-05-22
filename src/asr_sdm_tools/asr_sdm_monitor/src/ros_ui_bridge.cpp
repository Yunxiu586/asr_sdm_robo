#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QStringList>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QUrl>
#include <Qt>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace
{
constexpr int kVideoSlotCount = 4;
constexpr const char *kImageType = "sensor_msgs/msg/Image";
constexpr const char *kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr int kMaxPlotSamples = 600;
constexpr const char *kImuTopic = "/camera/camera/imu";
}

RosUiBridge::RosUiBridge(QObject *parent)
    : QObject(parent),
      ros_status_("Waiting for /diagnostics and /perception image topics ...")
{
    playback_timer_ = new QTimer(this);
    playback_timer_->setInterval(33);
    connect(playback_timer_, &QTimer::timeout, this, &RosUiBridge::playbackTick);

    node_ = std::make_shared<rclcpp::Node>("diagnostics_qml_ui_node");

    diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 20,
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
        {
            diagnosticsCallback(msg);
        });

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        kImuTopic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            imuCallback(msg);
        });

    // /perception video topics are discovered automatically from the ROS graph.
    video_topic_discovery_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(3000),
        [this]()
        {
            discoverVideoTopics();
        });


    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    ros_status_ = "Subscribed: /diagnostics, /camera/camera/imu; scanning /perception image topics";
    emit rosStatusChanged();

    discoverVideoTopics();

    ros_thread_ = std::thread([this]()
    {
        executor_->spin();
    });
}

RosUiBridge::~RosUiBridge()
{
    stopPlotRecording();
    setPlaybackPlaying(false);

    if (executor_) {
        executor_->cancel();
    }

    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
}

QString RosUiBridge::rosStatus() const
{
    return ros_status_;
}

QVariantMap RosUiBridge::cpuSummary() const
{
    return cpu_summary_;
}

QVariantList RosUiBridge::cpuHistory() const
{
    return cpu_history_;
}

QVariantList RosUiBridge::cpuCoreRows() const
{
    return cpu_core_rows_;
}

QVariantMap RosUiBridge::memorySummary() const
{
    return memory_summary_;
}

QVariantList RosUiBridge::memoryHistory() const
{
    return memory_history_;
}

QVariantList RosUiBridge::memoryRows() const
{
    return memory_rows_;
}

QVariantMap RosUiBridge::hddSummary() const
{
    return hdd_summary_;
}

QVariantList RosUiBridge::hddRows() const
{
    return hdd_rows_;
}

QVariantMap RosUiBridge::netSummary() const
{
    return net_summary_;
}

QVariantList RosUiBridge::netInHistory() const
{
    return net_in_history_;
}

QVariantList RosUiBridge::netOutHistory() const
{
    return net_out_history_;
}

QVariantList RosUiBridge::netInterfaceRows() const
{
    return net_interface_rows_;
}

QVariantMap RosUiBridge::ntpSummary() const
{
    return ntp_summary_;
}

QVariantList RosUiBridge::ntpRows() const
{
    return ntp_rows_;
}

QVariantList RosUiBridge::videoTopics() const
{
    return video_topics_;
}

QVariantList RosUiBridge::videoSlots() const
{
    QVariantList video_slot_list;
    for (int slot_index = 0; slot_index < kVideoSlotCount; ++slot_index) {
        const auto &slot = video_slots_[static_cast<size_t>(slot_index)];
        QVariantMap item;
        item[QStringLiteral("topic")] = slot.topic;
        item[QStringLiteral("topicType")] = slot.topic_type;
        item[QStringLiteral("status")] = slot.status;
        item[QStringLiteral("frameRevision")] = slot.frame_revision;
        video_slot_list.append(item);
    }
    return video_slot_list;
}

QString RosUiBridge::videoTopic0() const
{
    return video_slots_[0].topic;
}

QString RosUiBridge::videoTopic1() const
{
    return video_slots_[1].topic;
}

QString RosUiBridge::videoTopic2() const
{
    return video_slots_[2].topic;
}

QString RosUiBridge::videoTopic3() const
{
    return video_slots_[3].topic;
}

QString RosUiBridge::videoStatus0() const
{
    return video_slots_[0].status;
}

QString RosUiBridge::videoStatus1() const
{
    return video_slots_[1].status;
}

QString RosUiBridge::videoStatus2() const
{
    return video_slots_[2].status;
}

QString RosUiBridge::videoStatus3() const
{
    return video_slots_[3].status;
}

int RosUiBridge::videoFrame0Revision() const
{
    return video_slots_[0].frame_revision;
}

int RosUiBridge::videoFrame1Revision() const
{
    return video_slots_[1].frame_revision;
}

int RosUiBridge::videoFrame2Revision() const
{
    return video_slots_[2].frame_revision;
}

int RosUiBridge::videoFrame3Revision() const
{
    return video_slots_[3].frame_revision;
}

QImage RosUiBridge::videoFrameImage(int slotIndex) const
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return {};
    }

    std::lock_guard<std::mutex> lock(video_frame_mutex_);
    return video_slots_[static_cast<size_t>(slotIndex)].frame.copy();
}

QVariantList RosUiBridge::plotFieldOptions() const
{
    return plot_field_options_;
}

QVariantList RosUiBridge::imuPlotSamples() const
{
    return imu_plot_samples_;
}

QString RosUiBridge::plotStatus() const
{
    return plot_status_;
}

QString RosUiBridge::plotDataSource() const
{
    return plot_data_source_;
}

QVariantList RosUiBridge::recordedPlotFieldOptions() const
{
    return recorded_plot_field_options_;
}

QVariantList RosUiBridge::recordedPlotSamples() const
{
    return recorded_plot_samples_;
}

bool RosUiBridge::plotRecording() const
{
    return plot_recording_;
}

QString RosUiBridge::plotRecordingPath() const
{
    return plot_recording_path_;
}

QString RosUiBridge::recordedFilePath() const
{
    return recorded_file_path_;
}

QString RosUiBridge::recordedStatus() const
{
    return recorded_status_;
}

double RosUiBridge::playbackStartTimeMs() const
{
    return playback_start_time_ms_;
}

double RosUiBridge::playbackEndTimeMs() const
{
    return playback_end_time_ms_;
}

double RosUiBridge::playbackCurrentTimeMs() const
{
    return playback_current_time_ms_;
}

double RosUiBridge::playbackSpeed() const
{
    return playback_speed_;
}

bool RosUiBridge::playbackPlaying() const
{
    return playback_playing_;
}

void RosUiBridge::setPlotDataSource(const QString &dataSource)
{
    const QString normalized = dataSource.trimmed().toLower();
    const QString next = normalized == QStringLiteral("recorded") ? QStringLiteral("recorded") : QStringLiteral("live");
    if (plot_data_source_ == next) {
        return;
    }

    plot_data_source_ = next;
    emit plotDataSourceChanged();
}

QString RosUiBridge::defaultPlotRecordingPath() const
{
    const QString directory = QDir::homePath() + QStringLiteral("/asr_sdm_monitor_recordings");
    QDir().mkpath(directory);
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    return directory + QStringLiteral("/plot_") + stamp;
}

QString RosUiBridge::normalizeLocalPath(const QString &filePath)
{
    QString path = filePath.trimmed();
    if (path.startsWith(QStringLiteral("file:"))) {
        const QUrl url(path);
        if (url.isLocalFile()) {
            path = url.toLocalFile();
        }
    }

    if (path.startsWith(QStringLiteral("~/"))) {
        path = QDir::homePath() + path.mid(1);
    }

    if (path.isEmpty()) {
        return {};
    }

    return QDir::cleanPath(path);
}

bool RosUiBridge::startPlotRecording(const QString &filePath)
{
    QString path = normalizeLocalPath(filePath);
    if (path.isEmpty()) {
        path = defaultPlotRecordingPath();
    }

    stopPlotRecording();

    const QFileInfo info(path);
    const QString parent_dir = info.absolutePath();
    if (!QDir().mkpath(parent_dir)) {
        plot_status_ = QStringLiteral("Cannot create recording directory: %1").arg(parent_dir);
        emit plotStatusChanged();
        return false;
    }

    if (QFileInfo::exists(path)) {
        plot_status_ = QStringLiteral("Bag path already exists: %1").arg(path);
        emit plotStatusChanged();
        return false;
    }

    try {
        auto writer = std::make_unique<rosbag2_cpp::Writer>();
        writer->open(path.toStdString());

        {
            std::lock_guard<std::mutex> lock(plot_recording_mutex_);
            plot_bag_writer_ = std::move(writer);
            plot_recorded_message_count_ = 0;
            plot_recording_ = true;
            plot_recording_path_ = path;
        }

        plot_status_ = QStringLiteral("Recording bag: %1").arg(path);
        emit plotRecordingChanged();
        emit plotStatusChanged();
        return true;
    } catch (const std::exception &error) {
        plot_status_ = QStringLiteral("Cannot start bag recording: %1").arg(QString::fromUtf8(error.what()));
        emit plotStatusChanged();
        return false;
    }
}

void RosUiBridge::stopPlotRecording()
{
    bool was_recording = false;
    QString saved_path;
    size_t saved_count = 0;

    {
        std::lock_guard<std::mutex> lock(plot_recording_mutex_);
        was_recording = plot_recording_;
        saved_path = plot_recording_path_;
        saved_count = plot_recorded_message_count_;
        plot_recording_ = false;
        plot_bag_writer_.reset();
    }

    if (!was_recording) {
        return;
    }

    plot_status_ = QStringLiteral("Bag saved: %1 (%2 messages)").arg(saved_path).arg(saved_count);
    emit plotRecordingChanged();
    emit plotStatusChanged();
}

void RosUiBridge::writePlotRecordingSample(const sensor_msgs::msg::Imu &msg)
{
    std::lock_guard<std::mutex> lock(plot_recording_mutex_);
    if (!plot_recording_ || !plot_bag_writer_) {
        return;
    }

    try {
        rclcpp::Time timestamp(msg.header.stamp);
        if (timestamp.nanoseconds() <= 0 && node_) {
            timestamp = node_->now();
        }

        plot_bag_writer_->write(msg, std::string(kImuTopic), timestamp);
        ++plot_recorded_message_count_;
    } catch (const std::exception &error) {
        plot_recording_ = false;
        plot_status_ = QStringLiteral("Bag recording stopped: %1").arg(QString::fromUtf8(error.what()));
        plot_bag_writer_.reset();
        QMetaObject::invokeMethod(
            this,
            [this]()
            {
                emit plotRecordingChanged();
                emit plotStatusChanged();
            },
            Qt::QueuedConnection);
    }
}

bool RosUiBridge::openRecordedPlotFile(const QString &filePath)
{
    const QString path = normalizeLocalPath(filePath);
    if (path.isEmpty()) {
        recorded_status_ = QStringLiteral("Recorded file path is empty");
        emit recordedPlaybackChanged();
        return false;
    }

    setPlaybackPlaying(false);

    const bool ok = loadRecordedRosbag(path);

    if (ok) {
        setPlotDataSource(QStringLiteral("recorded"));
    }

    emit recordedPlaybackChanged();
    emit recordedPlotFieldOptionsChanged();
    emit recordedPlotSamplesChanged();
    emit playbackCurrentTimeMsChanged();
    return ok;
}

QVariantList RosUiBridge::plotFieldOptionsForPaths(const QStringList &paths)
{
    QVariantList result;
    const QVariantList candidates = defaultPlotFieldOptions();
    for (const QVariant &candidate : candidates) {
        const QVariantMap field = candidate.toMap();
        const QString path = field.value(QStringLiteral("path")).toString();
        if (paths.contains(path)) {
            result.append(field);
        }
    }
    return result;
}

bool RosUiBridge::loadRecordedRosbag(const QString &filePath)
{
    try {
        rosbag2_cpp::Reader reader;
        reader.open(filePath.toStdString());

        QVariantList samples;
        QStringList fieldPaths;
        const QVariantList candidates = defaultPlotFieldOptions();
        for (const QVariant &candidate : candidates) {
            fieldPaths.append(candidate.toMap().value(QStringLiteral("path")).toString());
        }

        rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
        double bagStartMs = -1.0;
        double bagEndMs = -1.0;

        while (reader.has_next()) {
            const auto bagMessage = reader.read_next();
            if (!bagMessage) {
                continue;
            }

            const double messageTimeMs = static_cast<double>(bagMessage->recv_timestamp) / 1000000.0;
            if (messageTimeMs >= 0.0) {
                if (bagStartMs < 0.0 || messageTimeMs < bagStartMs) {
                    bagStartMs = messageTimeMs;
                }
                if (bagEndMs < 0.0 || messageTimeMs > bagEndMs) {
                    bagEndMs = messageTimeMs;
                }
            }

            if (bagMessage->topic_name != kImuTopic) {
                continue;
            }

            sensor_msgs::msg::Imu imu;
            rclcpp::SerializedMessage serializedMessage(*bagMessage->serialized_data);
            serializer.deserialize_message(&serializedMessage, &imu);

            QVariantMap sample = sampleFromImu(imu, messageTimeMs);
            samples.append(sample);
        }

        if (bagStartMs < 0.0 || bagEndMs < 0.0) {
            recorded_status_ = QStringLiteral("No messages found in bag: %1").arg(filePath);
            return false;
        }

        std::sort(samples.begin(), samples.end(), [](const QVariant &a, const QVariant &b)
        {
            return a.toMap().value(QStringLiteral("absoluteTimeMs")).toDouble()
                   < b.toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
        });

        for (QVariant &item : samples) {
            QVariantMap sample = item.toMap();
            sample[QStringLiteral("relativeTime")] = (sample.value(QStringLiteral("absoluteTimeMs")).toDouble() - bagStartMs) / 1000.0;
            item = sample;
        }

        recorded_file_path_ = filePath;
        recorded_plot_samples_ = samples;
        recorded_plot_field_options_ = samples.isEmpty() ? QVariantList{} : plotFieldOptionsForPaths(fieldPaths);
        recorded_bag_start_time_ms_ = bagStartMs;
        recorded_bag_end_time_ms_ = bagEndMs;
        playback_start_time_ms_ = bagStartMs;
        playback_end_time_ms_ = bagEndMs;
        playback_current_time_ms_ = playback_start_time_ms_;
        recorded_status_ = samples.isEmpty()
                               ? QStringLiteral("Loaded bag: %1; no supported plot topics found").arg(filePath)
                               : QStringLiteral("Loaded bag: %1 (%2 IMU samples)").arg(filePath).arg(samples.size());
        return true;
    } catch (const std::exception &error) {
        recorded_status_ = QStringLiteral("Cannot load rosbag / MCAP: %1").arg(QString::fromUtf8(error.what()));
        return false;
    }
}

void RosUiBridge::updateRecordedPlaybackBounds()
{
    setPlaybackPlaying(false);

    if (recorded_plot_samples_.isEmpty()) {
        recorded_bag_start_time_ms_ = 0.0;
        recorded_bag_end_time_ms_ = 0.0;
        playback_start_time_ms_ = 0.0;
        playback_end_time_ms_ = 0.0;
        playback_current_time_ms_ = 0.0;
        return;
    }

    recorded_bag_start_time_ms_ = recorded_plot_samples_.first().toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
    recorded_bag_end_time_ms_ = recorded_plot_samples_.last().toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
    playback_start_time_ms_ = recorded_bag_start_time_ms_;
    playback_end_time_ms_ = recorded_bag_end_time_ms_;
    playback_current_time_ms_ = playback_start_time_ms_;
}

void RosUiBridge::setPlaybackStartTimeMs(double startTimeMs)
{
    if (recorded_bag_end_time_ms_ <= recorded_bag_start_time_ms_) {
        return;
    }

    const double upper = std::min(playback_end_time_ms_, recorded_bag_end_time_ms_);
    const double clamped = std::clamp(startTimeMs, recorded_bag_start_time_ms_, upper);
    if (std::abs(playback_start_time_ms_ - clamped) < 0.5) {
        return;
    }

    playback_start_time_ms_ = clamped;
    bool currentChanged = false;
    if (playback_current_time_ms_ < playback_start_time_ms_) {
        playback_current_time_ms_ = playback_start_time_ms_;
        currentChanged = true;
    }

    emit recordedPlaybackChanged();
    if (currentChanged) {
        emit playbackCurrentTimeMsChanged();
    }
}

void RosUiBridge::setPlaybackEndTimeMs(double endTimeMs)
{
    if (recorded_bag_end_time_ms_ <= recorded_bag_start_time_ms_) {
        return;
    }

    const double lower = std::max(playback_start_time_ms_, recorded_bag_start_time_ms_);
    const double clamped = std::clamp(endTimeMs, lower, recorded_bag_end_time_ms_);
    if (std::abs(playback_end_time_ms_ - clamped) < 0.5) {
        return;
    }

    playback_end_time_ms_ = clamped;
    bool currentChanged = false;
    if (playback_current_time_ms_ > playback_end_time_ms_) {
        playback_current_time_ms_ = playback_end_time_ms_;
        currentChanged = true;
    }

    emit recordedPlaybackChanged();
    if (currentChanged) {
        emit playbackCurrentTimeMsChanged();
    }
}

void RosUiBridge::setPlaybackCurrentTimeMs(double currentTimeMs)
{
    const double clamped = std::clamp(currentTimeMs, playback_start_time_ms_, playback_end_time_ms_);
    if (std::abs(playback_current_time_ms_ - clamped) < 0.5) {
        return;
    }

    playback_current_time_ms_ = clamped;
    emit playbackCurrentTimeMsChanged();
}

void RosUiBridge::setPlaybackSpeed(double speed)
{
    const double next = std::clamp(speed, 0.05, 20.0);
    if (std::abs(playback_speed_ - next) < 1e-6) {
        return;
    }

    playback_speed_ = next;
    emit playbackSpeedChanged();
}

void RosUiBridge::setPlaybackPlaying(bool playing)
{
    if (playing && playback_end_time_ms_ <= playback_start_time_ms_) {
        playing = false;
    }

    if (playback_playing_ == playing) {
        return;
    }

    playback_playing_ = playing;
    if (playback_timer_) {
        if (playback_playing_) {
            playback_last_tick_ = std::chrono::steady_clock::now();
            playback_timer_->start();
        } else {
            playback_timer_->stop();
        }
    }

    emit playbackPlayingChanged();
}

void RosUiBridge::playbackTick()
{
    if (!playback_playing_) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsedMs = std::chrono::duration<double, std::milli>(now - playback_last_tick_).count();
    playback_last_tick_ = now;

    double next = playback_current_time_ms_ + elapsedMs * playback_speed_;
    if (next >= playback_end_time_ms_) {
        next = playback_end_time_ms_;
        playback_current_time_ms_ = next;
        emit playbackCurrentTimeMsChanged();
        setPlaybackPlaying(false);
        return;
    }

    playback_current_time_ms_ = next;
    emit playbackCurrentTimeMsChanged();
}

void RosUiBridge::discoverVideoTopics()
{
    if (!node_) {
        return;
    }

    QStringList discovered_names;
    QMap<QString, QString> discovered_types;

    const auto topics_and_types = node_->get_topic_names_and_types();
    for (const auto &entry : topics_and_types) {
        const QString topic_name = QString::fromStdString(entry.first).trimmed();
        for (const std::string &type_string : entry.second) {
            const QString topic_type = QString::fromStdString(type_string).trimmed();
            if (isPerceptionImageTopic(topic_name, topic_type)) {
                discovered_names.append(topic_name);
                discovered_types.insert(topic_name, topic_type);
                break;
            }
        }
    }

    QMetaObject::invokeMethod(
        this,
        [this, discovered_names, discovered_types]()
        {
            applyDiscoveredVideoTopics(discovered_names, discovered_types);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::setVideoTopic(int slotIndex, const QString &topicName)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    const QString normalized_topic = topicName.trimmed();

    if (!normalized_topic.isEmpty() && !video_topic_names_.contains(normalized_topic)) {
        return;
    }

    for (int other_slot_index = 0; other_slot_index < kVideoSlotCount; ++other_slot_index) {
        if (other_slot_index == slotIndex) {
            continue;
        }

        auto &other_slot = video_slots_[static_cast<size_t>(other_slot_index)];
        if (!normalized_topic.isEmpty() && other_slot.topic == normalized_topic) {
            stopVideoSlot(other_slot_index);
            other_slot.topic.clear();
            other_slot.topic_type.clear();
            other_slot.status = QStringLiteral("No topic selected");
            emitVideoSlotChanged(other_slot_index);
        }
    }

    auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
    if (slot.topic == normalized_topic) {
        return;
    }

    stopVideoSlot(slotIndex);
    slot.topic = normalized_topic;
    slot.topic_type = video_topic_types_.value(normalized_topic);
    slot.status = normalized_topic.isEmpty() ? QStringLiteral("No topic selected")
                                           : QStringLiteral("Waiting for video frame");

    emitVideoSlotChanged(slotIndex);

    if (!normalized_topic.isEmpty()) {
        startVideoSlot(slotIndex);
    }
}

void RosUiBridge::diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    QMetaObject::invokeMethod(
        this,
        [this]()
        {
            ros_status_ = "Receiving: /diagnostics";
            emit rosStatusChanged();
        },
        Qt::QueuedConnection);

    for (const auto &status : msg->status) {
        const QString name = QString::fromStdString(status.name).toLower();

        if (name.contains("cpu usage")) {
            updateCpu(status);
        } else if (name.contains("memory usage")) {
            updateMemory(status);
        } else if (name.contains("hdd usage")) {
            updateHdd(status);
        } else if (name.contains("network usage")) {
            updateNet(status);
        } else if (name.contains("ntp offset")) {
            updateNtp(status);
        }
    }
}

QString RosUiBridge::valueForKey(const diagnostic_msgs::msg::DiagnosticStatus &status, const QString &key)
{
    for (const auto &item : status.values) {
        if (QString::fromStdString(item.key) == key) {
            return QString::fromStdString(item.value);
        }
    }
    return {};
}

double RosUiBridge::extractNumber(const QString &text)
{
    static const std::regex pattern(R"((-?\d+(?:\.\d+)?))");
    std::smatch match;
    const std::string input = text.toStdString();
    if (std::regex_search(input, match, pattern)) {
        return std::stod(match.str(1));
    }
    return 0.0;
}

QString RosUiBridge::formatNumber(double value, int precision)
{
    return QString::number(value, 'f', precision);
}

QString RosUiBridge::levelToText(unsigned char level)
{
    switch (level) {
    case 0:
        return "OK";
    case 1:
        return "WARN";
    case 2:
        return "ERROR";
    case 3:
        return "STALE";
    default:
        return "UNKNOWN";
    }
}

void RosUiBridge::appendHistory(QVariantList &history, double value, int maxPoints)
{
    history.append(value);
    while (history.size() > maxPoints) {
        history.removeFirst();
    }
}

void RosUiBridge::updateCpu(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    struct CoreData {
        int index = -1;
        QString status;
        QString clock;
        QString user;
        QString system;
        QString idle;
        double usage = 0.0;
        double clockValue = 0.0;
    };

    std::map<int, CoreData> cores;
    static const std::regex core_pattern(R"(Core\s+(\d+)\s+(.+))");

    for (const auto &item : status.values) {
        std::smatch match;
        const std::string key = item.key;
        if (!std::regex_match(key, match, core_pattern)) {
            continue;
        }

        const int idx = std::stoi(match.str(1));
        const QString field = QString::fromStdString(match.str(2));
        auto &core = cores[idx];
        core.index = idx;

        const QString value = QString::fromStdString(item.value);
        if (field == "Status") {
            core.status = value;
        } else if (field == "Clock Speed") {
            core.clock = value;
            core.clockValue = extractNumber(value);
        } else if (field == "User") {
            core.user = value;
        } else if (field == "System") {
            core.system = value;
        } else if (field == "Idle") {
            core.idle = value;
            core.usage = std::clamp(100.0 - extractNumber(value), 0.0, 100.0);
        }
    }

    QVariantList rows;
    std::vector<double> usages;
    std::vector<double> clocks;
    double max_usage = 0.0;

    for (const auto &[index, core] : cores) {
        Q_UNUSED(index);
        QVariantMap row;
        row["core"] = QString::number(core.index);
        row["usage"] = formatNumber(core.usage, 1) + "%";
        row["clock"] = core.clock;
        row["user"] = core.user;
        row["system"] = core.system;
        row["idle"] = core.idle;
        row["status"] = core.status;
        rows.append(row);

        usages.push_back(core.usage);
        clocks.push_back(core.clockValue);
        max_usage = std::max(max_usage, core.usage);
    }

    const double avg_usage = usages.empty()
                                 ? 0.0
                                 : std::accumulate(usages.begin(), usages.end(), 0.0) / usages.size();
    const double avg_clock = clocks.empty()
                                 ? 0.0
                                 : std::accumulate(clocks.begin(), clocks.end(), 0.0) / clocks.size();

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["avgUsage"] = formatNumber(avg_usage, 1) + "%";
    summary["maxUsage"] = formatNumber(max_usage, 1) + "%";
    summary["avgClock"] = formatNumber(avg_clock, 0) + " MHz";
    summary["load1"] = valueForKey(status, "Load Average (1min)");
    summary["load5"] = valueForKey(status, "Load Average (5min)");
    summary["load15"] = valueForKey(status, "Load Average (15min)");
    summary["coreCount"] = static_cast<int>(rows.size());

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, avg_usage]()
        {
            cpu_summary_ = summary;
            appendHistory(cpu_history_, avg_usage / 100.0);
            cpu_core_rows_ = rows;
            emit cpuSummaryChanged();
            emit cpuHistoryChanged();
            emit cpuCoreRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateMemory(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    const QString total_physical = valueForKey(status, "Total Memory (Physical)");
    const QString used_physical = valueForKey(status, "Used Memory (Physical)");
    const QString free_physical = valueForKey(status, "Free Memory (Physical)");
    const QString total_swap = valueForKey(status, "Total Memory (Swap)");
    const QString used_swap = valueForKey(status, "Used Memory (Swap)");
    const QString free_swap = valueForKey(status, "Free Memory (Swap)");
    const QString total_all = valueForKey(status, "Total Memory");
    const QString used_all = valueForKey(status, "Used Memory");
    const QString free_all = valueForKey(status, "Free Memory");

    const double total_physical_num = extractNumber(total_physical);
    const double used_physical_num = extractNumber(used_physical);
    const double usage_percent = total_physical_num > 0.0
                                     ? (used_physical_num / total_physical_num) * 100.0
                                     : 0.0;

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["usedPhysical"] = used_physical;
    summary["totalPhysical"] = total_physical;
    summary["freePhysical"] = free_physical;
    summary["usedSwap"] = used_swap;
    summary["totalSwap"] = total_swap;
    summary["usagePercent"] = formatNumber(usage_percent, 1) + "%";
    summary["updateStatus"] = valueForKey(status, "Update Status");

    QVariantList rows;
    rows.append(QVariantMap{{"item", "Physical"}, {"total", total_physical}, {"used", used_physical}, {"free", free_physical}});
    rows.append(QVariantMap{{"item", "Swap"}, {"total", total_swap}, {"used", used_swap}, {"free", free_swap}});
    rows.append(QVariantMap{{"item", "Combined"}, {"total", total_all}, {"used", used_all}, {"free", free_all}});

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, usage_percent]()
        {
            memory_summary_ = summary;
            appendHistory(memory_history_, usage_percent / 100.0);
            memory_rows_ = rows;
            emit memorySummaryChanged();
            emit memoryHistoryChanged();
            emit memoryRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateHdd(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    QVariantList rows;
    QVariantMap current_disk;
    bool has_disk = false;
    double worst_use = 0.0;

    for (const auto &item : status.values) {
        const QString key = QString::fromStdString(item.key);
        const QString value = QString::fromStdString(item.value);

        if (key.contains("Name") && key.startsWith("Disk")) {
            if (has_disk) {
                rows.append(current_disk);
            }
            current_disk.clear();
            current_disk["disk"] = value;
            has_disk = true;
        } else if (has_disk && key.contains("Size")) {
            current_disk["size"] = value;
        } else if (has_disk && key.contains("Available")) {
            current_disk["available"] = value;
        } else if (has_disk && key.contains("Use")) {
            current_disk["use"] = value;
            worst_use = std::max(worst_use, extractNumber(value));
        } else if (has_disk && key.contains("Status")) {
            current_disk["status"] = value;
        } else if (has_disk && key.contains("Mount Point")) {
            current_disk["mount"] = value;
        }
    }

    if (has_disk) {
        rows.append(current_disk);
    }

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["diskCount"] = static_cast<int>(rows.size());
    summary["worstUse"] = formatNumber(worst_use, 0) + "%";

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows]()
        {
            hdd_summary_ = summary;
            hdd_rows_ = rows;
            emit hddSummaryChanged();
            emit hddRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateNet(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    QVariantList rows;
    QVariantMap current_if;
    bool has_interface = false;
    double total_in = 0.0;
    double total_out = 0.0;
    int total_errors = 0;
    QStringList interfaces;

    for (const auto &item : status.values) {
        const QString key = QString::fromStdString(item.key);
        const QString value = QString::fromStdString(item.value);

        if (key == "Interface Name") {
            if (has_interface) {
                rows.append(current_if);
            }
            current_if.clear();
            current_if["interface"] = value;
            interfaces << value;
            has_interface = true;
        } else if (has_interface && key == "State") {
            current_if["state"] = value;
        } else if (has_interface && key == "Input Traffic") {
            current_if["input"] = value;
            total_in += extractNumber(value);
        } else if (has_interface && key == "Output Traffic") {
            current_if["output"] = value;
            total_out += extractNumber(value);
        } else if (has_interface && key == "MTU") {
            current_if["mtu"] = value;
        } else if (has_interface && key == "Total received MB") {
            current_if["totalRx"] = value;
        } else if (has_interface && key == "Total transmitted MB") {
            current_if["totalTx"] = value;
        } else if (has_interface && key == "Collisions") {
            current_if["collisions"] = value;
        } else if (has_interface && key == "Rx Errors") {
            current_if["rxErrors"] = value;
            total_errors += static_cast<int>(extractNumber(value));
        } else if (has_interface && key == "Tx Errors") {
            current_if["txErrors"] = value;
            total_errors += static_cast<int>(extractNumber(value));
        }
    }

    if (has_interface) {
        rows.append(current_if);
    }

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["input"] = formatNumber(total_in, 4) + " MB/s";
    summary["output"] = formatNumber(total_out, 4) + " MB/s";
    summary["interfaces"] = interfaces.join(", ");
    summary["interfaceCount"] = interfaces.size();
    summary["errors"] = total_errors;

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, total_in, total_out]()
        {
            net_summary_ = summary;
            appendHistory(net_in_history_, total_in);
            appendHistory(net_out_history_, total_out);
            net_interface_rows_ = rows;
            emit netSummaryChanged();
            emit netInHistoryChanged();
            emit netOutHistoryChanged();
            emit netInterfaceRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateNtp(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    const QString offset = valueForKey(status, "Offset (us)");
    const QString tolerance = valueForKey(status, "Offset tolerance (us)");
    const QString error_tolerance = valueForKey(status, "Offset tolerance (us) for Error");

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["offset"] = offset + " us";
    summary["tolerance"] = tolerance + " us";
    summary["errorTolerance"] = error_tolerance + " us";

    QVariantList rows;
    rows.append(QVariantMap{{"name", "Offset (us)"}, {"value", offset}});
    rows.append(QVariantMap{{"name", "Tolerance (us)"}, {"value", tolerance}});
    rows.append(QVariantMap{{"name", "Error Tolerance (us)"}, {"value", error_tolerance}});

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows]()
        {
            ntp_summary_ = summary;
            ntp_rows_ = rows;
            emit ntpSummaryChanged();
            emit ntpRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::handleVideoTopicsMessage(const std_msgs::msg::String::SharedPtr msg)
{
    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if (!document.isObject()) {
        return;
    }

    const QJsonArray topics = document.object().value(QStringLiteral("topics")).toArray();
    QStringList discovered_names;
    QMap<QString, QString> discovered_types;

    for (const QJsonValue &value : topics) {
        const QJsonObject topic_object = value.toObject();
        const QString name = topic_object.value(QStringLiteral("name")).toString().trimmed();
        const QString type = topic_object.value(QStringLiteral("type")).toString().trimmed();
        if (!isPerceptionImageTopic(name, type)) {
            continue;
        }

        discovered_names.append(name);
        discovered_types.insert(name, type);
    }

    QMetaObject::invokeMethod(
        this,
        [this, discovered_names, discovered_types]()
        {
            applyDiscoveredVideoTopics(discovered_names, discovered_types);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::applyDiscoveredVideoTopics(const QStringList &topicNames, const QMap<QString, QString> &topicTypes)
{
    QStringList discovered_names = topicNames;
    discovered_names.removeDuplicates();
    discovered_names.sort(Qt::CaseSensitive);

    QMap<QString, QString> discovered_types;
    QVariantList topic_items;
    for (const QString &name : discovered_names) {
        const QString type = topicTypes.value(name);
        if (!isPerceptionImageTopic(name, type)) {
            continue;
        }
        discovered_types.insert(name, type);
        topic_items.append(name);
    }

    if (video_topic_names_ == discovered_names && video_topic_types_ == discovered_types) {
        return;
    }

    video_topic_names_ = discovered_names;
    video_topic_types_ = discovered_types;
    video_topics_ = topic_items;
    emit videoTopicsChanged();

    for (int slot_index = 0; slot_index < kVideoSlotCount; ++slot_index) {
        auto &slot = video_slots_[static_cast<size_t>(slot_index)];
        if (!slot.topic.isEmpty() && !video_topic_names_.contains(slot.topic)) {
            stopVideoSlot(slot_index);
            slot.topic.clear();
            slot.topic_type.clear();
            slot.status = QStringLiteral("No topic selected");
            emitVideoSlotChanged(slot_index);
        }
    }

}

bool RosUiBridge::isPerceptionImageTopic(const QString &topicName, const QString &topicType)
{
    return topicName.startsWith(QStringLiteral("/perception"))
           && (topicType == QString::fromLatin1(kImageType)
               || topicType == QString::fromLatin1(kCompressedImageType));
}

void RosUiBridge::handleVideoSlotStatusMessage(int slotIndex, const std_msgs::msg::String::SharedPtr msg)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if (!document.isObject()) {
        return;
    }

    const QJsonObject object = document.object();
    const QString selected_topic = object.value(QStringLiteral("selected_topic")).toString().trimmed();
    const QString topic_type = object.value(QStringLiteral("topic_type")).toString().trimmed();
    const QString status = object.value(QStringLiteral("status")).toString().trimmed();

    QMetaObject::invokeMethod(
        this,
        [this, slotIndex, selected_topic, topic_type, status]()
        {
            auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
            slot.topic = selected_topic;
            slot.topic_type = topic_type;
            if (!status.isEmpty()) {
                slot.status = status;
            }
            emitVideoSlotChanged(slotIndex);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::publishVideoSelection(int slotIndex, const QString &topicName)
{
    if (!video_select_pub_) {
        return;
    }

    QJsonObject payload;
    payload.insert(QStringLiteral("slot"), slotIndex);
    payload.insert(QStringLiteral("topic"), topicName);

    std_msgs::msg::String msg;
    msg.data = QJsonDocument(payload).toJson(QJsonDocument::Compact).toStdString();
    video_select_pub_->publish(msg);
}

void RosUiBridge::stopVideoSlot(int slotIndex)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
    slot.image_sub.reset();
    slot.compressed_sub.reset();
    slot.frame_revision = 0;

    {
        std::lock_guard<std::mutex> lock(video_frame_mutex_);
        slot.frame = QImage();
    }

    if (slot.topic.isEmpty()) {
        slot.status = QStringLiteral("No topic selected");
    }
    emitVideoSlotChanged(slotIndex);
}

void RosUiBridge::startVideoSlot(int slotIndex)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
    slot.image_sub.reset();
    slot.compressed_sub.reset();

    if (slot.topic.isEmpty()) {
        slot.status = QStringLiteral("No topic selected");
        emitVideoSlotChanged(slotIndex);
        return;
    }

    const std::string selected_topic = slot.topic.toStdString();
    const rclcpp::SensorDataQoS qos;

    slot.status = QStringLiteral("Waiting for video frame");

    if (slot.topic_type == QString::fromLatin1(kImageType) || slot.topic_type.isEmpty()) {
        slot.image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
            selected_topic, qos,
            [this, slotIndex](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                imageCallback(slotIndex, msg);
            });
    }

    if (slot.topic_type == QString::fromLatin1(kCompressedImageType) || slot.topic_type.isEmpty()) {
        slot.compressed_sub = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
            selected_topic, qos,
            [this, slotIndex](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                compressedImageCallback(slotIndex, msg);
            });
    }

    emitVideoSlotChanged(slotIndex);
}

void RosUiBridge::imageCallback(int slotIndex, const sensor_msgs::msg::Image::SharedPtr msg)
{
    QString error;
    const QImage image = imageMessageToQImage(*msg, &error);
    if (image.isNull()) {
        updateVideoStatus(slotIndex, error.isEmpty() ? QStringLiteral("Unable to decode image") : error);
        return;
    }

    const QString status = QStringLiteral("%1x%2 %3")
                               .arg(msg->width)
                               .arg(msg->height)
                               .arg(QString::fromStdString(msg->encoding));
    updateVideoFrame(slotIndex, image, status);
}

void RosUiBridge::compressedImageCallback(int slotIndex, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    QImage image;
    if (!image.loadFromData(msg->data.data(), static_cast<int>(msg->data.size()))) {
        updateVideoStatus(slotIndex, QStringLiteral("Unable to decode compressed image"));
        return;
    }

    const QString format = QString::fromStdString(msg->format).trimmed();
    const QString status = format.isEmpty()
                               ? QStringLiteral("Compressed image")
                               : QStringLiteral("Compressed image (%1)").arg(format);
    updateVideoFrame(slotIndex, image, status);
}

void RosUiBridge::updateVideoFrame(int slotIndex, const QImage &image, const QString &status)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    const QImage image_copy = image.copy();

    QMetaObject::invokeMethod(
        this,
        [this, slotIndex, image_copy, status]()
        {
            auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
            {
                std::lock_guard<std::mutex> lock(video_frame_mutex_);
                slot.frame = image_copy;
            }

            ++slot.frame_revision;
            slot.status = status;
            emitVideoSlotChanged(slotIndex);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateVideoStatus(int slotIndex, const QString &status)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    QMetaObject::invokeMethod(
        this,
        [this, slotIndex, status]()
        {
            auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
            slot.status = status;
            emitVideoSlotChanged(slotIndex);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::emitVideoSlotChanged(int slotIndex)
{
    if (slotIndex == 0) {
        emit videoSlot0Changed();
    } else if (slotIndex == 1) {
        emit videoSlot1Changed();
    } else if (slotIndex == 2) {
        emit videoSlot2Changed();
    } else if (slotIndex == 3) {
        emit videoSlot3Changed();
    }

    emit videoSlotsChanged();
}

QVariantList RosUiBridge::defaultPlotFieldOptions()
{
    QVariantList fields;

    auto append_field = [&fields](const QString &fieldName, const QString &label, const QString &unit)
    {
        QVariantMap field;
        field[QStringLiteral("field")] = fieldName;
        field[QStringLiteral("path")] = plotTopicPath(fieldName);
        field[QStringLiteral("label")] = label;
        field[QStringLiteral("unit")] = unit;
        fields.append(field);
    };

    append_field(QStringLiteral("angular_velocity.x"), QStringLiteral("angular_velocity.x"), QStringLiteral("rad/s"));
    append_field(QStringLiteral("angular_velocity.y"), QStringLiteral("angular_velocity.y"), QStringLiteral("rad/s"));
    append_field(QStringLiteral("angular_velocity.z"), QStringLiteral("angular_velocity.z"), QStringLiteral("rad/s"));
    append_field(QStringLiteral("linear_acceleration.x"), QStringLiteral("linear_acceleration.x"), QStringLiteral("m/s^2"));
    append_field(QStringLiteral("linear_acceleration.y"), QStringLiteral("linear_acceleration.y"), QStringLiteral("m/s^2"));
    append_field(QStringLiteral("linear_acceleration.z"), QStringLiteral("linear_acceleration.z"), QStringLiteral("m/s^2"));

    return fields;
}

QString RosUiBridge::plotTopicPath(const QString &fieldName)
{
    return QStringLiteral("/camera/camera/imu.%1").arg(fieldName);
}

QVariantMap RosUiBridge::sampleFromImu(const sensor_msgs::msg::Imu &msg, double fallbackAbsoluteTimeMs)
{
    double absoluteTimeMs = (static_cast<double>(msg.header.stamp.sec) * 1000.0)
                            + (static_cast<double>(msg.header.stamp.nanosec) * 1e-6);
    if (absoluteTimeMs <= 0.0 && fallbackAbsoluteTimeMs >= 0.0) {
        absoluteTimeMs = fallbackAbsoluteTimeMs;
    }

    QVariantMap sample;
    sample[QStringLiteral("stamp")] = absoluteTimeMs / 1000.0;
    sample[QStringLiteral("absoluteTimeMs")] = absoluteTimeMs;
    sample[QStringLiteral("/camera/camera/imu.angular_velocity.x")] = msg.angular_velocity.x;
    sample[QStringLiteral("/camera/camera/imu.angular_velocity.y")] = msg.angular_velocity.y;
    sample[QStringLiteral("/camera/camera/imu.angular_velocity.z")] = msg.angular_velocity.z;
    sample[QStringLiteral("/camera/camera/imu.linear_acceleration.x")] = msg.linear_acceleration.x;
    sample[QStringLiteral("/camera/camera/imu.linear_acceleration.y")] = msg.linear_acceleration.y;
    sample[QStringLiteral("/camera/camera/imu.linear_acceleration.z")] = msg.linear_acceleration.z;
    return sample;
}

void RosUiBridge::activatePublishedPlotFields(const QVariantMap &sample)
{
    bool changed = false;
    const QVariantList candidates = defaultPlotFieldOptions();

    for (const QVariant &candidate : candidates) {
        const QVariantMap field = candidate.toMap();
        const QString path = field.value(QStringLiteral("path")).toString();

        if (path.isEmpty() || active_plot_field_paths_.contains(path) || !sample.contains(path)) {
            continue;
        }

        active_plot_field_paths_.append(path);
        plot_field_options_.append(field);
        changed = true;
    }

    if (changed) {
        emit plotFieldOptionsChanged();
    }
}

void RosUiBridge::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    const QVariantMap sample = sampleFromImu(*msg);
    const double stamp = sample.value(QStringLiteral("stamp")).toDouble();
    writePlotRecordingSample(*msg);

    QMetaObject::invokeMethod(
        this,
        [this, sample, stamp]()
        {
            if (plot_start_time_ < 0.0) {
                plot_start_time_ = stamp;
            }

            activatePublishedPlotFields(sample);

            QVariantMap stored_sample = sample;
            stored_sample[QStringLiteral("relativeTime")] = stamp - plot_start_time_;
            imu_plot_samples_.append(stored_sample);
            while (imu_plot_samples_.size() > kMaxPlotSamples) {
                imu_plot_samples_.removeFirst();
            }

            bool recording_now = false;
            QString recording_path;
            size_t recorded_count = 0;
            {
                std::lock_guard<std::mutex> lock(plot_recording_mutex_);
                recording_now = plot_recording_;
                recording_path = plot_recording_path_;
                recorded_count = plot_recorded_message_count_;
            }

            plot_status_ = recording_now
                               ? QStringLiteral("Recording bag: %1 (%2 messages)").arg(recording_path).arg(recorded_count)
                               : QStringLiteral("Receiving: /camera/camera/imu (%1 samples)").arg(imu_plot_samples_.size());
            emit imuPlotSamplesChanged();
            emit plotStatusChanged();
        },
        Qt::QueuedConnection);
}

QImage RosUiBridge::imageMessageToQImage(const sensor_msgs::msg::Image &msg, QString *errorMessage)
{
    if (msg.width == 0 || msg.height == 0) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Invalid image size");
        }
        return {};
    }

    const qsizetype required_size = static_cast<qsizetype>(msg.step) * static_cast<qsizetype>(msg.height);
    if (required_size <= 0 || static_cast<qsizetype>(msg.data.size()) < required_size) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Image data is incomplete");
        }
        return {};
    }

    const uchar *data = msg.data.data();
    const int width = static_cast<int>(msg.width);
    const int height = static_cast<int>(msg.height);
    const int bytes_per_line = static_cast<int>(msg.step);
    const QString encoding = QString::fromStdString(msg.encoding).toLower();

    if (encoding == QStringLiteral("rgb8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGB888).copy();
    }

    if (encoding == QStringLiteral("bgr8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGB888).rgbSwapped();
    }

    if (encoding == QStringLiteral("rgba8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGBA8888).copy();
    }

    if (encoding == QStringLiteral("bgra8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGBA8888).rgbSwapped();
    }

    if (encoding == QStringLiteral("mono8") || encoding == QStringLiteral("8uc1")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_Grayscale8).copy();
    }

    if (errorMessage) {
        *errorMessage = QStringLiteral("Unsupported image encoding: %1").arg(QString::fromStdString(msg.encoding));
    }
    return {};
}
