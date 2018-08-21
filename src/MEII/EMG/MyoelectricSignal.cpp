#include <MEII/EMG/MyoelectricSignal.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    MyoelectricSignal::MyoelectricSignal(AnalogInput::Channel ai_channel, std::size_t buffer_capacity,
        std::size_t hp_filter_order, double hp_filter_cutoff,
        std::size_t lp_filter_order, double lp_filter_cutoff,
        std::size_t tkeo_lp_filter_order, double tkeo_lp_filter_cutoff) :
        ai_channel_(ai_channel),
        raw_(0.0),
        demean_(0.0),
        envelope_(0.0),
        tkeo_envelope_(0.0),
        buffer_capacity_(buffer_capacity),
        raw_buffer_(buffer_capacity_),
        dm_buffer_(buffer_capacity_),
        env_buffer_(buffer_capacity_),
        tkeo_env_buffer_(buffer_capacity_),
        hp_filter_(Butterworth(hp_filter_order, hp_filter_cutoff, Butterworth::Highpass, 100)),
		full_rect_(true),
        lp_filter_(Butterworth(lp_filter_order, lp_filter_cutoff, Butterworth::Lowpass, 100)),
		half_rect_(false),
		tkeo_full_rect_(true),
        tkeo_lp_filter_(Butterworth(tkeo_lp_filter_order, tkeo_lp_filter_cutoff, Butterworth::Lowpass, 100)),
		tkeo_half_rect_(false)
    { }

	bool MyoelectricSignal::on_enable() {
		return true;
	}

	bool MyoelectricSignal::on_disable() {
		return true;
	}

    void MyoelectricSignal::update() {
        raw_ = ai_channel_.get_value();
        demean_ = hp_filter_.update(raw_);
        envelope_ = half_rect_.update(lp_filter_.update(full_rect_.update(demean_)));
        tkeo_envelope_ = tkeo_half_rect_.update(tkeo_lp_filter_.update(tkeo_full_rect_.update(tkeo_.update(demean_))));
    }

    void MyoelectricSignal::update_and_buffer() {
        update();
        push_buffer();
    }

    void MyoelectricSignal::clear_buffer() {
        raw_buffer_.clear();
        dm_buffer_.clear();
        env_buffer_.clear();
        tkeo_env_buffer_.clear();
    }

    void MyoelectricSignal::resize_buffer(std::size_t capacity) {
        buffer_capacity_ = capacity;
        raw_buffer_.resize(buffer_capacity_);
        dm_buffer_.resize(buffer_capacity_);
        env_buffer_.resize(buffer_capacity_);
        tkeo_env_buffer_.resize(buffer_capacity_);
    }

    std::size_t MyoelectricSignal::get_buffer_capacity() const {
        return buffer_capacity_;
    }

    std::size_t MyoelectricSignal::get_buffer_size() const {
        std::size_t buffer_size = raw_buffer_.size();
        if (buffer_size != dm_buffer_.size() || buffer_size != env_buffer_.size() || buffer_size != tkeo_env_buffer_.size())
            LOG(Warning) << "Myoelectric signal on channel " << ai_channel_.get_channel_number() << " has mismatched buffer sizes. Returning size of raw buffer.";
        return buffer_size;
    }

    bool MyoelectricSignal::is_buffer_full() {
        return raw_buffer_.full() && dm_buffer_.full() && env_buffer_.full() && tkeo_env_buffer_.full();
    }

    void MyoelectricSignal::reset_signal_processing() {
        hp_filter_.reset();
        full_rect_.reset();
        lp_filter_.reset();
		half_rect_.reset();
        tkeo_.reset();
        tkeo_full_rect_.reset();
        tkeo_lp_filter_.reset();
		tkeo_half_rect_.reset();
    }

    double MyoelectricSignal::get_raw() const {
        return raw_;
    }

    double MyoelectricSignal::get_demean() const {
        return demean_;
    }

    double MyoelectricSignal::get_envelope() const {
        return envelope_;
    }

    double MyoelectricSignal::get_tkeo_envelope() const {
        return tkeo_envelope_;
    }

    std::vector<double> MyoelectricSignal::get_raw_buffer_data(std::size_t window_size) const {
        if (raw_buffer_.size() < window_size) {
            LOG(Warning) << "Myoelectric signal on channel " << ai_channel_.get_channel_number() << " cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << raw_buffer_.size() << ".";
            window_size = raw_buffer_.size();
        }
        std::vector<double> raw_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            raw_window[i] = raw_buffer_[raw_buffer_.size() + i - window_size];
        }
        return raw_window;
    }

    std::vector<double> MyoelectricSignal::get_dm_buffer_data(std::size_t window_size) const {
        if (dm_buffer_.size() < window_size) {
            LOG(Warning) << "Myoelectric signal on channel " << ai_channel_.get_channel_number() << " cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << dm_buffer_.size() << ".";
            window_size = dm_buffer_.size();
        }
        std::vector<double> dm_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            dm_window[i] = dm_buffer_[dm_buffer_.size() + i - window_size];
        }
        return dm_window;
    }

    std::vector<double> MyoelectricSignal::get_env_buffer_data(std::size_t window_size) const {
        if (env_buffer_.size() < window_size) {
            LOG(Warning) << "Myoelectric signal on channel " << ai_channel_.get_channel_number() << " cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << env_buffer_.size() << ".";
            window_size = env_buffer_.size();
        }
        std::vector<double> env_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            env_window[i] = env_buffer_[env_buffer_.size() + i - window_size];
        }
        return env_window;
    }

    std::vector<double> MyoelectricSignal::get_tkeo_env_buffer_data(std::size_t window_size) const {
        if (tkeo_env_buffer_.size() < window_size) {
            LOG(Warning) << "Myoelectric signal on channel " << ai_channel_.get_channel_number() << " cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << tkeo_env_buffer_.size() << ".";
            window_size = tkeo_env_buffer_.size();
        }
        std::vector<double> tkeo_env_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            tkeo_env_window[i] = tkeo_env_buffer_[tkeo_env_buffer_.size() + i - window_size];
        }
        return tkeo_env_window;
    }

    void MyoelectricSignal::push_buffer() {
        raw_buffer_.push_back(raw_);
        dm_buffer_.push_back(demean_);
        env_buffer_.push_back(envelope_);
        tkeo_env_buffer_.push_back(tkeo_envelope_);
    }

} // namespace meii