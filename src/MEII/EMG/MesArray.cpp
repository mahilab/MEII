#include <MEII/EMG/MesArray.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>

using namespace mel;

namespace meii {

    MesArray::MesArray(std::vector<AnalogInput::Channel> ai_channels, std::size_t buffer_capacity,
        std::size_t hp_filter_order, double hp_filter_cutoff,
        std::size_t lp_filter_order, double lp_filter_cutoff,
        std::size_t tkeo_lp_filter_order, double tkeo_lp_filter_cutoff) :
        mes_count_(ai_channels.size()),
        raw_(mes_count_),
        demean_(mes_count_),
        envelope_(mes_count_),
        tkeo_envelope_(mes_count_),
		tkeo_envelope_mean_(1),
        buffer_capacity_(buffer_capacity),
        raw_buffer_(buffer_capacity),
        dm_buffer_(buffer_capacity),
        env_buffer_(buffer_capacity),
        tkeo_env_buffer_(buffer_capacity),
		tkeo_env_mean_buffer_(buffer_capacity)
    {
        for (std::size_t i = 0; i < mes_count_; ++i) {
            mes_.emplace_back(ai_channels[i], buffer_capacity, hp_filter_order, hp_filter_cutoff, lp_filter_order, lp_filter_cutoff, tkeo_lp_filter_order, tkeo_lp_filter_cutoff);
            channel_numbers_.push_back(ai_channels[i].get_channel_number());
        }
    }

    void MesArray::update() {
        for (std::size_t i = 0; i < mes_count_; ++i) {
            mes_[i].update();
            raw_[i] = mes_[i].get_raw();
            demean_[i] = mes_[i].get_demean();
            envelope_[i] = mes_[i].get_envelope();
            tkeo_envelope_[i] = mes_[i].get_tkeo_envelope();
        }
		calculate_means();
    }

    void MesArray::update_and_buffer() {
        for (std::size_t i = 0; i < mes_count_; ++i) {
            mes_[i].update_and_buffer();
            raw_[i] = mes_[i].get_raw();
            demean_[i] = mes_[i].get_demean();
            envelope_[i] = mes_[i].get_envelope();
            tkeo_envelope_[i] = mes_[i].get_tkeo_envelope();
        }
		calculate_means();
        push_buffer();
    }

    void MesArray::clear_buffer() {
        for (std::size_t i = 0; i < mes_count_; ++i) {
            mes_[i].clear_buffer();
        }
		raw_buffer_.clear();
		dm_buffer_.clear();
		env_buffer_.clear();
		tkeo_env_buffer_.clear();
		tkeo_env_mean_buffer_.clear();
    }

    void MesArray::resize_buffer(std::size_t capacity) {
        for (std::size_t i = 0; i < mes_count_; ++i) {
            mes_[i].resize_buffer(capacity);
        }
        buffer_capacity_ = capacity;
        raw_buffer_.resize(buffer_capacity_);
        dm_buffer_.resize(buffer_capacity_);
        env_buffer_.resize(buffer_capacity_);
        tkeo_env_buffer_.resize(buffer_capacity_);
		tkeo_env_mean_buffer_.resize(buffer_capacity_);
    }

    std::size_t MesArray::get_buffer_capacity() const {
        return buffer_capacity_;
    }

    std::size_t MesArray::get_buffer_size() const {
        std::size_t buffer_size;
        if (!mes_.empty())
            buffer_size = mes_[0].get_buffer_size();
        else
            return 0;
        for (std::size_t i = 0; i < mes_count_; ++i) {
            if (buffer_size != mes_[i].get_buffer_size())
                LOG(Warning) << "Myoelectric signal array has mismatched buffer sizes. Returning buffer size of first channel.";
        }
        return buffer_size;
    }

    bool MesArray::is_buffer_full() {
        bool full = true;
        for (std::size_t i = 0; i < mes_count_; ++i) {
            if (!mes_[i].is_buffer_full())
                full = false;
        }
        return full;
    }

    void MesArray::reset_signal_processing() {
        for (std::size_t i = 0; i < mes_count_; ++i) {
            mes_[i].reset_signal_processing();
        }
    }

    std::size_t MesArray::size() const {
        return mes_count_;
    }

    const std::vector<double>& MesArray::get_raw() const {
        return raw_;
    }

    const std::vector<double>& MesArray::get_demean() const {
        return demean_;
    }

    const std::vector<double>& MesArray::get_envelope() const {
        return envelope_;
    }

    const std::vector<double>& MesArray::get_tkeo_envelope() const {
        return tkeo_envelope_;
    }

	const std::vector<double>& MesArray::get_tkeo_envelope_mean() const {
		return tkeo_envelope_mean_;
	}

    std::vector<std::vector<double>> MesArray::get_raw_buffer_data(std::size_t window_size) const {
        if (get_buffer_size() < window_size) {
            LOG(Warning) << "Myoelectric signal array cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << get_buffer_size() << ".";
            window_size = get_buffer_size();
        }
        std::vector<std::vector<double>> raw_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            raw_window[i] = raw_buffer_[raw_buffer_.size() + i - window_size];
        }
        return raw_window;
    }

    std::vector<std::vector<double>> MesArray::get_dm_buffer_data(std::size_t window_size) const {
        if (get_buffer_size() < window_size) {
            LOG(Warning) << "Myoelectric signal array cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << get_buffer_size() << ".";
            window_size = get_buffer_size();
        }
        std::vector<std::vector<double>> dm_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            dm_window[i] = dm_buffer_[dm_buffer_.size() + i - window_size];
        }
        return dm_window;
    }

    std::vector<std::vector<double>> MesArray::get_env_buffer_data(std::size_t window_size) const {
        if (get_buffer_size() < window_size) {
            LOG(Warning) << "Myoelectric signal array cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << get_buffer_size() << ".";
            window_size = get_buffer_size();
        }
        std::vector<std::vector<double>> env_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            env_window[i] = env_buffer_[env_buffer_.size() + i - window_size];
        }
        return env_window;
    }

    std::vector<std::vector<double>> MesArray::get_tkeo_env_buffer_data(std::size_t window_size) const {
        if (get_buffer_size() < window_size) {
            LOG(Warning) << "Myoelectric signal array cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << get_buffer_size() << ".";
            window_size = get_buffer_size();
        }
        
        std::vector<std::vector<double>> tkeo_env_window(window_size);
        for (std::size_t i = 0; i < window_size; ++i) {
            tkeo_env_window[i] = tkeo_env_buffer_[tkeo_env_buffer_.size() + i - window_size];
        }
        return tkeo_env_window;
    }

	std::vector<std::vector<double>> MesArray::get_tkeo_env_mean_buffer_data(std::size_t window_size) const {
		if (get_buffer_size() < window_size) {
			LOG(Warning) << "Myoelectric signal array cannot return requested amount of buffer data until buffer is of size " << window_size << ". Returning vector of size " << get_buffer_size() << ".";
			window_size = get_buffer_size();
		}

		std::vector<std::vector<double>> tkeo_env_mean_window(window_size);
		for (std::size_t i = 0; i < window_size; ++i) {
			tkeo_env_mean_window[i] = tkeo_env_mean_buffer_[tkeo_env_mean_buffer_.size() + i - window_size];
		}
		return tkeo_env_mean_window;
	}


    std::vector<double> MesArray::get_single_raw_buffer_data(std::size_t mes_index, std::size_t window_size) const {
        if (mes_index >= mes_count_) {
            LOG(Warning) << "Invalid MES index provided for MesArray. Returning empty vector.";
            return std::vector<double>();
        }
        return mes_[mes_index].get_raw_buffer_data(window_size);
    }


    std::vector<double> MesArray::get_single_dm_buffer_data(std::size_t mes_index, std::size_t window_size) const {
        if (mes_index >= mes_count_) {
            LOG(Warning) << "Invalid MES index provided for MesArray. Returning empty vector.";
            return std::vector<double>();
        }
        return mes_[mes_index].get_dm_buffer_data(window_size);
    }


    std::vector<double> MesArray::get_single_env_buffer_data(std::size_t mes_index, std::size_t window_size) const {
        if (mes_index >= mes_count_) {
            LOG(Warning) << "Invalid MES index provided for MesArray. Returning empty vector.";
            return std::vector<double>();
        }
        return mes_[mes_index].get_env_buffer_data(window_size);
    }


    std::vector<double> MesArray::get_single_tkeo_env_buffer_data(std::size_t mes_index, std::size_t window_size) const {
        if (mes_index >= mes_count_) {
            LOG(Warning) << "Invalid MES index provided for MesArray. Returning empty vector.";
            return std::vector<double>();
        }
        return mes_[mes_index].get_tkeo_env_buffer_data(window_size);
    }

    void MesArray::push_buffer() {
        raw_buffer_.push_back(raw_);
        dm_buffer_.push_back(demean_);
        env_buffer_.push_back(envelope_);
        tkeo_env_buffer_.push_back(tkeo_envelope_);
		tkeo_env_mean_buffer_.push_back(tkeo_envelope_mean_);
    }

	void MesArray::calculate_means() {
		tkeo_envelope_mean_[0] = mel::mean(tkeo_envelope_);
	}

} // namespace meii