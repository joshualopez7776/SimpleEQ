/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>

#include <array>

/**
 * @brief A FIFO buffer for storing and retrieving audio or vector data.
 *
 * This template class implements a First-In-First-Out (FIFO) buffer with a fixed
 * capacity, designed for audio processing or vector data storage. It supports
 * preparing buffers, pushing data, and pulling data in a thread-safe manner.
 *
 * @tparam T The type of data stored (either juce::AudioBuffer<float> or std::vector<float>).
 */
template<typename T>
struct Fifo
{
    /**
     * @brief Prepares the FIFO for audio buffers.
     *
     * Configures each buffer in the FIFO to handle the specified number of channels
     * and samples, clearing existing content and avoiding reallocation where possible.
     *
     * @param numChannels Number of audio channels.
     * @param numSamples Number of samples per channel.
     */
    void prepare(int numChannels, int numSamples)
    {
        static_assert(std::is_same_v<T, juce::AudioBuffer<float>>,
            "prepare(numChannels, numSamples) should only be used when the Fifo is holding juce::AudioBuffer<float>");
        for (auto& buffer : buffers)
        {
            buffer.setSize(
                numChannels,
                numSamples,
                false, //clear everything
                true,  //including the extra space
                true   //avlid reallocating if you can
            );

            buffer.clear();
        }
    }

    /**
     * @brief Prepares the FIFO for vector data.
     *
     * Resizes each buffer in the FIFO to hold the specified number of elements,
     * initializing them to zero.
     *
     * @param numElements Number of elements in each vector.
     */
    void prepare(size_t numElements)
    {
        static_assert(std::is_same_v<T, std::vector<float>>,
            "prepare(numElements) should only be used when the Fifo is holding std::vector<float>");

        for (auto& buffer : buffers)
        {
            buffer.clear();
            buffer.resize(numElements, 0);
        }
    }

    /**
     * @brief Pushes data into the FIFO.
     *
     * Attempts to write one element to the FIFO at the next available position.
     *
     * @param t The data to push.
     * @return True if the push was successful, false if the FIFO is full.
     */
    bool push(const T& t)
    {
        auto write = fifo.write(1);
        if (write.blockSize1 > 0)
        {
            buffers[write.startIndex1] = t;
            return true;
        }

        return false;
    }

    /**
     * @brief Pulls data from the FIFO.
     *
     * Retrieves one element from the FIFO if available.
     *
     * @param t Reference to store the retrieved data.
     * @return True if data was pulled, false if the FIFO is empty.
     */
    bool pull(T& t)
    {
        auto read = fifo.read(1);
        if (read.blockSize1 > 0)
        {
            t = buffers[read.startIndex1];
            return true;
        }

        return false;
    }

    /**
     * @brief Returns the number of elements available for reading.
     *
     * @return The number of elements ready to be pulled.
     */
    int getNumAvailableForReading() const
    {
        return fifo.getNumReady();
    }
private:
    static constexpr int Capacity = 30; // Fixed FIFO capacity
    std::array<T, Capacity> buffers; // Array of buffers for FIFO storage
    juce::AbstractFifo fifo{ Capacity }; // JUCE FIFO implementation
};

/**
 * @brief Enum defining audio channel indices.
 */
enum Channel
{
    Right, // effectively 0
    Left // effectively 1
};

/**
 * @brief Manages a FIFO for a single audio channel.
 *
 * Stores samples from a specific channel (left or right) into a FIFO for
 * real-time processing or analysis, such as visualization in the plugin's GUI.
 *
 * @tparam BlockType The type of audio buffer (typically juce::AudioBuffer<float>).
 */
template <typename BlockType>
struct SingleChannelSampleFifo
{
    /**
     * @brief Constructor. Initializes the FIFO for a specific channel.
     *
     * @param ch The channel to process (Left or Right).
     */
    SingleChannelSampleFifo(Channel ch) : channelToUse(ch)
    {
        prepared.set(false);
    }

    /**
     * @brief Updates the FIFO with samples from the input buffer.
     *
     * Copies samples from the specified channel of the input buffer into the FIFO.
     *
     * @param buffer The input audio buffer containing the channel data.
     */
    void update(const BlockType& buffer)
    {
        jassert(prepared.get());
        jassert(buffer.getNumChannels() > channelToUse);
        auto* channelPtr = buffer.getReadPointer(channelToUse);

        // Push each sample into the FIFO
        for (int i = 0; i < buffer.getNumSamples(); i++)
        {
            pushNextSampleIntoFifo(channelPtr[i]);
        }
    }

    /**
     * @brief Prepares the FIFO for processing.
     *
     * Configures the buffer size and initializes the FIFO for a single channel.
     *
     * @param bufferSize The number of samples to store.
     */
    void prepare(int bufferSize)
    {
        prepared.set(false);
        size.set(bufferSize);
        
        // Configure the buffer for single-channel audio
        bufferToFill.setSize(
            1,           // channel
            bufferSize,  // num samples
            false,       // keepExistingContent
            true,        // clear extra space
            true         // avoid reallocation
        );

        audioBufferFifo.prepare(1, bufferSize);
        fifoIndex = 0;
        prepared.set(true);
    }

    //==============================================================================
    /**
     * @brief Returns the number of complete buffers available in the FIFO.
     *
     * @return Number of buffers ready for reading.
     */
    int getNumCompleteBuffersAvailable() const { return audioBufferFifo.getNumAvailableForReading(); }

    /**
     * @brief Checks if the FIFO is prepared.
     *
     * @return True if the FIFO is ready for processing.
     */
    bool isPrepared() const { return prepared.get(); }

    /**
     * @brief Returns the buffer size.
     *
     * @return The number of samples per buffer.
     */
    int getSize() const { return size.get(); }
    //==============================================================================

    /**
     * @brief Retrieves an audio buffer from the FIFO.
     *
     * @param buf The buffer to fill with FIFO data.
     * @return True if a buffer was retrieved, false if the FIFO is empty.
     */
    bool getAudioBuffer(BlockType& buf) { return audioBufferFifo.pull(buf); }
private:
    Channel channelToUse; // Channel to process (Left or Right)
    int fifoIndex = 0; // Current index in the buffer
    Fifo<BlockType> audioBufferFifo; // FIFO for storing audio buffers
    BlockType bufferToFill; // Temporary buffer for building FIFO entries
    juce::Atomic<bool> prepared = false; // Tracks preparation state
    juce::Atomic<int> size = 0; // Buffer size

    /**
     * @brief Pushes a single sample into the FIFO.
     *
     * Adds a sample to the temporary buffer and pushes it to the FIFO when full.
     *
     * @param sample The audio sample to push.
     */
    void pushNextSampleIntoFifo(float sample)
    {
        if (fifoIndex == bufferToFill.getNumSamples())
        {
            auto ok = audioBufferFifo.push(bufferToFill);

            juce::ignoreUnused(ok);

            fifoIndex = 0;
        }

        bufferToFill.setSample(0, fifoIndex, sample);
        ++fifoIndex;
    }
};

/**
 * @brief Enum defining filter slope options for cut filters.
 */
enum Slope
{
    Slope_12,
    Slope_24,
    Slope_36,
    Slope_48

};

/**
 * @brief Stores settings for the EQ filter chain.
 */
struct ChainSettings
{
    float peakFreq{ 0 }, peakGainInDecibels{ 0 }, peakQuality{ 1.f };
    float lowCutFreq{ 0 }, highCutFreq{ 0 };
    
    Slope lowCutSlope{ Slope::Slope_12 }, highCutSlope{ Slope::Slope_12 };

    bool lowCutBypassed{ false }, peakBypassed{ false }, highCutBypassed{ false };
};

/**
 * @brief Retrieves the current chain settings from the parameter tree.
 *
 * @param apvts The AudioProcessorValueTreeState containing parameter values.
 * @return The ChainSettings structure with current values.
 */
ChainSettings getChainSettings(juce::AudioProcessorValueTreeState& apvts);

using Filter = juce::dsp::IIR::Filter<float>;

using CutFilter = juce::dsp::ProcessorChain<Filter, Filter, Filter, Filter>;

using MonoChain = juce::dsp::ProcessorChain<CutFilter, Filter, CutFilter>;

enum ChainPositions
{
    LowCut,
    Peak,
    HighCut
};

using Coefficients = Filter::CoefficientsPtr;

/**
 * @brief Updates filter coefficients.
 *
 * Replaces old coefficients with new ones for a filter.
 *
 * @param old The current coefficients.
 * @param replacements The new coefficients.
 */
void updateCoefficients(Coefficients& old, const Coefficients& replacements);

/**
 * @brief Creates coefficients for the peak filter.
 *
 * @param chainSettings The current EQ settings.
 * @param sampleRate The audio sample rate.
 * @return The peak filter coefficients.
 */
Coefficients makePeakFilter(const ChainSettings& chainSettings, double sampleRate);

/**
 * @brief Updates a filter in a processor chain.
 *
 * @tparam Index The index of the filter in the chain.
 * @tparam ChainType The type of processor chain.
 * @tparam CoefficientType The type of coefficients.
 * @param chain The processor chain to update.
 * @param coefficients The new coefficients.
 */
template<int Index, typename ChainType, typename CoefficientType>
void update(ChainType& chain, const CoefficientType& coefficients)
{
    updateCoefficients(chain.template get<Index>().coefficients, coefficients[Index]);
    chain.template setBypassed<Index>(false);
}

/**
 * @brief Updates a cut filter based on slope settings.
 *
 * Configures the appropriate number of filter stages based on the slope (12, 24, 36, or 48 dB/octave).
 *
 * @tparam ChainType The type of processor chain.
 * @tparam CoefficientType The type of coefficients.
 * @param leftLowCut The low-cut filter chain to update.
 * @param cutCoefficients The new coefficients.
 * @param lowCutSlope The slope setting for the filter.
 */
template<typename ChainType, typename CoefficientType>
void updateCutFilter(ChainType& leftLowCut,
    const CoefficientType& cutCoefficients,
    const Slope& lowCutSlope)
{

    leftLowCut.template setBypassed<0>(true);
    leftLowCut.template setBypassed<1>(true);
    leftLowCut.template setBypassed<2>(true);
    leftLowCut.template setBypassed<3>(true);

    // Enable the appropriate number of stages based on slope
    switch (lowCutSlope)
    {
    case Slope_48:
    {
        update<3>(leftLowCut, cutCoefficients);
    }
    case Slope_36:
    {
        update<2>(leftLowCut, cutCoefficients);
    }
    case Slope_24:
    {
        update<1>(leftLowCut, cutCoefficients);
    }
    case Slope_12:
    {
        update<0>(leftLowCut, cutCoefficients);
    }
    }
}

/**
 * @brief Creates coefficients for the low-cut filter.
 *
 * Uses Butterworth filter design for high-order high-pass filtering.
 *
 * @param chainSettings The current EQ settings.
 * @param sampleRate The audio sample rate.
 * @return The low-cut filter coefficients.
 */
inline auto makeLowCutFilter(const ChainSettings& chainSettings, double sampleRate)
{
    return juce::dsp::FilterDesign<float>::designIIRHighpassHighOrderButterworthMethod(chainSettings.lowCutFreq,
        sampleRate, 2 * (chainSettings.lowCutSlope + 1));
}

/**
 * @brief Creates coefficients for the high-cut filter.
 *
 * Uses Butterworth filter design for high-order low-pass filtering.
 *
 * @param chainSettings The current EQ settings.
 * @param sampleRate The audio sample rate.
 * @return The high-cut filter coefficients.
 */
inline auto makeHighCutFilter(const ChainSettings& chainSettings, double sampleRate)
{
    return juce::dsp::FilterDesign<float>::designIIRLowpassHighOrderButterworthMethod(chainSettings.highCutFreq,
        sampleRate, 2 * (chainSettings.highCutSlope + 1));
}

//==============================================================================

/**
 * @brief The main audio processor for the SimpleEQ plugin.
 *
 * Manages a 3-band equalizer with low-cut, peak, and high-cut filters, supporting
 * variable slopes (12, 24, 36, 48 dB/octave) and bypass options. Includes FIFOs
 * for real-time audio analysis (e.g., for GUI visualization).
 */
class SimpleEQAudioProcessor  : public juce::AudioProcessor
{
public:
    //==============================================================================
    /**
     * @brief Constructor.
     *
     * Initializes the processor with a parameter layout and default settings.
     */
    SimpleEQAudioProcessor();

    /**
     * @brief Destructor.
     *
     * Cleans up resources, relying on JUCE's smart pointers for memory management.
     */
    ~SimpleEQAudioProcessor() override;

    //==============================================================================
    /**
     * @brief Prepares the processor for playback.
     *
     * Initializes filters and FIFOs based on the sample rate and block size.
     *
     * @param sampleRate The audio sample rate (e.g., 44100 Hz).
     * @param samplesPerBlock The expected number of samples per block.
     */
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;

    /**
     * @brief Releases resources allocated during prepareToPlay.
     */
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    /**
     * @brief Checks if the bus layout is supported.
     *
     * Ensures the plugin supports stereo input and output.
     *
     * @param layouts The proposed bus layout.
     * @return True if the layout is supported.
     */
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    /**
     * @brief Processes an audio buffer through the EQ chain.
     *
     * Applies low-cut, peak, and high-cut filters to the buffer and updates FIFOs
     * for analysis.
     *
     * @param buffer The audio buffer to process.
     * @param midiMessages MIDI messages (unused in this plugin).
     */
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================

    /**
     * @brief Creates the editor for the plugin's GUI.
     *
     * @return A pointer to the editor instance.
     */
    juce::AudioProcessorEditor* createEditor() override;

    /**
     * @brief Indicates if the plugin has an editor.
     *
     * @return True, as this plugin provides a GUI.
     */
    bool hasEditor() const override;

    //==============================================================================
    /**
     * @brief Returns the plugin's name.
     *
     * @return The name "SimpleEQ".
     */
    const juce::String getName() const override;

    /**
     * @brief Indicates if the plugin accepts MIDI input.
     *
     * @return False, as this EQ does not process MIDI.
     */
    bool acceptsMidi() const override;

    /**
     * @brief Indicates if the plugin produces MIDI output.
     *
     * @return False, as this EQ does not produce MIDI.
     */
    bool producesMidi() const override;

    /**
     * @brief Indicates if the plugin is a MIDI effect.
     *
     * @return False, as this is an audio processor.
     */
    bool isMidiEffect() const override;

    /**
     * @brief Returns the tail length of the plugin's output.
     *
     * @return 0.0, as this EQ does not introduce delay.
     */
    double getTailLengthSeconds() const override;

    //==============================================================================
    /**
     * @brief Returns the number of preset programs.
     *
     * @return 1, as this plugin does not use presets.
     */
    int getNumPrograms() override;

    /**
     * @brief Returns the current program index.
     *
     * @return 0, as there is only one program.
     */
    int getCurrentProgram() override;

    /**
     * @brief Sets the current program (no-op).
     *
     * @param index The program index.
     */
    void setCurrentProgram (int index) override;

    /**
     * @brief Returns the name of a program.
     *
     * @param index The program index.
     * @return An empty string, as presets are not used.
     */
    const juce::String getProgramName (int index) override;

    /**
     * @brief Changes the program name (no-op).
     *
     * @param index The program index.
     * @param newName The new name.
     */
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    /**
     * @brief Saves the plugin's state.
     *
     * Serializes parameter values to a memory block for preset recall.
     *
     * @param destData The memory block to store the state.
     */
    void getStateInformation (juce::MemoryBlock& destData) override;

    /**
     * @brief Restores the plugin's state.
     *
     * Deserializes parameter values from a memory block.
     *
     * @param data The memory block containing the state.
     * @param sizeInBytes The size of the data.
     */
    void setStateInformation (const void* data, int sizeInBytes) override;

    /**
     * @brief Creates the parameter layout for the EQ.
     *
     * Defines parameters for frequency, gain, Q, and slope for each band.
     *
     * @return The parameter layout.
     */
    static juce::AudioProcessorValueTreeState::ParameterLayout
        createParameterLayout();
    juce::AudioProcessorValueTreeState apvts 
    {
        *this, nullptr, "Parameters", createParameterLayout()
    };

    using BlockType = juce::AudioBuffer<float>;
    SingleChannelSampleFifo<BlockType> leftChannelFifo{ Channel::Left };
    SingleChannelSampleFifo<BlockType> rightChannelFifo{ Channel::Right };
    
private:
    
    MonoChain leftChain, rightChain;

    /**
     * @brief Updates the peak filter coefficients.
     *
     * @param chainSettings The current EQ settings.
     */
    void updatePeakFilter(const ChainSettings& chainSettings);

    /**
     * @brief Updates the low-cut filter coefficients and bypass states.
     *
     * @param chainSettings The current EQ settings.
     */
    void updateLowCutFilters(const ChainSettings& chainSettings);

    /**
     * @brief Updates the high-cut filter coefficients and bypass states.
     *
     * @param chainSettings The current EQ settings.
     */
    void updateHighCutFilters(const ChainSettings& chainSettings);

    /**
     * @brief Updates all filters for both channels.
     */
    void updateFilters();

    juce::dsp::Oscillator<float> osc;
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SimpleEQAudioProcessor)
};
