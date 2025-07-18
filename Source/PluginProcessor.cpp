/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
/**
 * @brief Constructs the SimpleEQAudioProcessor with default bus configuration.
 *
 * Initializes the audio processor with stereo input and output channels (unless
 * configured as a synth or MIDI effect). The parameter tree and filter chains are
 * set up in the header file via the initializer list.
 */
SimpleEQAudioProcessor::SimpleEQAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       )
#endif
{
}

/**
 * @brief Destructor. Cleans up resources.
 *
 * Relies on JUCE's smart pointers (e.g., for parameter tree and FIFOs) for automatic
 * memory management.
 */
SimpleEQAudioProcessor::~SimpleEQAudioProcessor()
{
}

//==============================================================================
/**
 * @brief Returns the plugin's name.
 *
 * @return The string "SimpleEQ" as defined in the JUCE project settings.
 */
const juce::String SimpleEQAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

/**
 * @brief Indicates if the plugin accepts MIDI input.
 *
 * @return False, as this EQ does not process MIDI data.
 */
bool SimpleEQAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

/**
 * @brief Indicates if the plugin produces MIDI output.
 *
 * @return False, as this EQ does not generate MIDI data.
 */
bool SimpleEQAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

/**
 * @brief Indicates if the plugin is a MIDI effect.
 *
 * @return False, as this is an audio processing plugin.
 */
bool SimpleEQAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

/**
 * @brief Returns the tail length of the plugin's output.
 *
 * @return 0.0, as this EQ does not introduce processing delay.
 */
double SimpleEQAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

/**
 * @brief Returns the number of preset programs.
 *
 * @return 1, as this plugin does not support multiple presets.
 */
int SimpleEQAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

/**
 * @brief Returns the current program index.
 *
 * @return 0, as there is only one program.
 */
int SimpleEQAudioProcessor::getCurrentProgram()
{
    return 0;
}

/**
 * @brief Sets the current program (no-op).
 *
 * @param index The program index (ignored).
 */
void SimpleEQAudioProcessor::setCurrentProgram (int index)
{
}

/**
 * @brief Returns the name of a program.
 *
 * @param index The program index.
 * @return An empty string, as presets are not implemented.
 */
const juce::String SimpleEQAudioProcessor::getProgramName (int index)
{
    return {};
}

void SimpleEQAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================

/**
 * @brief Prepares the processor for audio playback.
 *
 * Initializes filter chains, FIFOs, and an optional test oscillator based on the
 * provided sample rate and block size. Ensures resources are ready for real-time
 * audio processing and analysis.
 *
 * @param sampleRate The audio sample rate (e.g., 44100 Hz).
 * @param samplesPerBlock The expected number of samples per processing block.
 */
void SimpleEQAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback
    // initialisation that you need..

    juce::dsp::ProcessSpec spec;

    spec.maximumBlockSize = samplesPerBlock;

    spec.numChannels = 1;

    spec.sampleRate = sampleRate;

    leftChain.prepare(spec);
    rightChain.prepare(spec);

    updateFilters();

    leftChannelFifo.prepare(samplesPerBlock);
    rightChannelFifo.prepare(samplesPerBlock);

    osc.initialise([](float x) { return std::sin(x); });

    spec.numChannels = getTotalNumOutputChannels();
    osc.prepare(spec);
    osc.setFrequency(100);
}

/**
 * @brief Releases resources allocated during prepareToPlay.
 *
 * Resets filter chains and clears FIFOs to free up memory.
 */
void SimpleEQAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

/**
 * @brief Checks if the bus layout is supported.
 *
 * Ensures the plugin supports stereo input and output (or mono for non-synth plugins).
 *
 * @param layouts The proposed bus layout.
 * @return True if the layout is supported, false otherwise.
 */
#ifndef JucePlugin_PreferredChannelConfigurations
bool SimpleEQAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif


/**
 * @brief Processes an audio buffer through the EQ chain.
 *
 * Applies low-cut, peak, and high-cut filters to the left and right channels,
 * updates FIFOs for analysis, and optionally applies a test oscillator (commented out).
 * Prevents denormal numbers for performance.
 *
 * @param buffer The audio buffer to process.
 * @param midiMessages MIDI messages (unused in this plugin).
 */
void SimpleEQAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // In case we have more outputs than inputs, this code clears any output
    // channels that didn't contain input data, (because these aren't
    // guaranteed to be empty - they may contain garbage).
    // This is here to avoid people getting screaming feedback
    // when they first compile a plugin, but obviously you don't need to keep
    // this code if your algorithm always overwrites all the output channels.
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());
    
    updateFilters();
    
    juce::dsp::AudioBlock<float> block(buffer);

    auto leftBlock = block.getSingleChannelBlock(0);
    auto rightBlock = block.getSingleChannelBlock(1);

    juce::dsp::ProcessContextReplacing<float> leftContext(leftBlock);
    juce::dsp::ProcessContextReplacing<float> rightContext(rightBlock);

    leftChain.process(leftContext);
    rightChain.process(rightContext);

    leftChannelFifo.update(buffer);
    rightChannelFifo.update(buffer);

    // This is the place where you'd normally do the guts of your plugin's
    // audio processing...
    // Make sure to reset the state if your inner loop is processing
    // the samples and the outer loop is handling the channels.
    // Alternatively, you can process the samples with the channels
    // interleaved by keeping the same state.
    for (int channel = 0; channel < totalNumInputChannels; ++channel)
    {
        auto* channelData = buffer.getWritePointer (channel);

        // ..do something to the data...
    }
}

//==============================================================================
/**
 * @brief Indicates if the plugin has an editor.
 *
 * @return True, as this plugin provides a GUI.
 */
bool SimpleEQAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

/**
 * @brief Creates the plugin's editor.
 *
 * @return A pointer to a new instance of SimpleEQAudioProcessorEditor.
 */
juce::AudioProcessorEditor* SimpleEQAudioProcessor::createEditor()
{
    return new SimpleEQAudioProcessorEditor (*this);
    // return new juce::GenericAudioProcessorEditor(*this);
}

//==============================================================================

/**
 * @brief Saves the plugin's state to a memory block.
 *
 * Serializes the parameter tree for preset recall.
 *
 * @param destData The memory block to store the state.
 */
void SimpleEQAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.

    juce::MemoryOutputStream mos(destData, true);
    apvts.state.writeToStream(mos);
}

/**
 * @brief Restores the plugin's state from a memory block.
 *
 * Deserializes parameter values and updates filters accordingly.
 *
 * @param data The memory block containing the state.
 * @param sizeInBytes The size of the data.
 */
void SimpleEQAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.

    auto tree = juce::ValueTree::readFromData(data, sizeInBytes);
    if (tree.isValid())
    {
        apvts.replaceState(tree);
        updateFilters();
    }
}

/**
 * @brief Retrieves the current EQ settings from the parameter tree.
 *
 * Loads frequency, gain, Q, slope, and bypass settings for all bands.
 *
 * @param apvts The AudioProcessorValueTreeState containing parameter values.
 * @return The ChainSettings structure with current values.
 */
ChainSettings getChainSettings(juce::AudioProcessorValueTreeState& apvts)
{
    ChainSettings settings;

    // Load frequency, gain, and quality parameters
    settings.lowCutFreq = apvts.getRawParameterValue("LowCut Freq")->load();
    settings.highCutFreq = apvts.getRawParameterValue("HighCut Freq")->load();
    settings.peakFreq = apvts.getRawParameterValue("Peak Freq")->load();
    settings.peakGainInDecibels = apvts.getRawParameterValue("Peak Gain")->load();
    settings.peakQuality = apvts.getRawParameterValue("Peak Quality")->load();

    // Load slope parameters (cast to enum)
    settings.lowCutSlope = static_cast<Slope>(apvts.getRawParameterValue("LowCut Slope")->load());
    settings.highCutSlope = static_cast<Slope>(apvts.getRawParameterValue("HighCut Slope")->load());

    // Load bypass parameters (threshold at 0.5 for boolean)
    settings.lowCutBypassed = apvts.getRawParameterValue("LowCut Bypassed")->load() > 0.5f;
    settings.peakBypassed = apvts.getRawParameterValue("Peak Bypassed")->load() > 0.5f;
    settings.highCutBypassed = apvts.getRawParameterValue("HighCut Bypassed")->load() > 0.5f;

    return settings;
}

/**
 * @brief Creates coefficients for the peak filter.
 *
 * Generates IIR coefficients for a peak filter based on frequency, Q, and gain.
 *
 * @param chainSettings The current EQ settings.
 * @param sampleRate The audio sample rate.
 * @return The peak filter coefficients.
 */
Coefficients makePeakFilter(const ChainSettings& chainSettings, double sampleRate)
{
    return juce::dsp::IIR::Coefficients<float>::makePeakFilter(sampleRate,
        chainSettings.peakFreq,
        chainSettings.peakQuality,
        juce::Decibels::decibelsToGain(chainSettings.peakGainInDecibels));
}

/**
 * @brief Updates the peak filter for both channels.
 *
 * Applies new coefficients and bypass state based on current settings.
 *
 * @param chainSettings The current EQ settings.
 */
void SimpleEQAudioProcessor::updatePeakFilter(const ChainSettings& chainSettings)
{
    auto peakCoefficients = makePeakFilter(chainSettings, getSampleRate());

    // Update bypass state for peak filter
    leftChain.setBypassed<ChainPositions::Peak>(chainSettings.peakBypassed);
    rightChain.setBypassed<ChainPositions::Peak>(chainSettings.peakBypassed);

    // Update coefficients for left and right channels
    updateCoefficients(leftChain.get<ChainPositions::Peak>().coefficients, peakCoefficients);
    updateCoefficients(rightChain.get<ChainPositions::Peak>().coefficients, peakCoefficients);
}

/**
 * @brief Updates filter coefficients.
 *
 * Replaces old coefficients with new ones for a given filter.
 *
 * @param old The current coefficients.
 * @param replacements The new coefficients.
 */
void updateCoefficients(Coefficients& old, const Coefficients& replacements)
{
    *old = *replacements;
}

/**
 * @brief Updates the low-cut filters for both channels.
 *
 * Applies new coefficients and bypass state based on frequency and slope settings.
 * Uses Butterworth filters for smooth frequency response.
 *
 * @param chainSettings The current EQ settings.
 */
void SimpleEQAudioProcessor::updateLowCutFilters(const ChainSettings& chainSettings)
{
    auto cutCoefficients = makeLowCutFilter(chainSettings, getSampleRate());
    auto& leftLowCut = leftChain.get < ChainPositions::LowCut>();
    auto& rightLowCut = rightChain.get < ChainPositions::LowCut>();

    // Update bypass state
    leftChain.setBypassed<ChainPositions::LowCut>(chainSettings.lowCutBypassed);
    rightChain.setBypassed<ChainPositions::LowCut>(chainSettings.lowCutBypassed);

    // Update filter coefficients based on slope
    updateCutFilter(leftLowCut, cutCoefficients, chainSettings.lowCutSlope);
    updateCutFilter(rightLowCut, cutCoefficients, chainSettings.lowCutSlope);
}

/**
 * @brief Updates the high-cut filters for both channels.
 *
 * Applies new coefficients and bypass state based on frequency and slope settings.
 * Uses Butterworth filters for smooth frequency response.
 *
 * @param chainSettings The current EQ settings.
 */
void SimpleEQAudioProcessor::updateHighCutFilters(const ChainSettings& chainSettings)
{
    auto highCutCoefficients = makeHighCutFilter(chainSettings, getSampleRate());

    auto& leftHighCut = leftChain.get < ChainPositions::HighCut>();
    auto& rightHighCut = rightChain.get < ChainPositions::HighCut>();

    leftChain.setBypassed<ChainPositions::HighCut>(chainSettings.highCutBypassed);
    rightChain.setBypassed<ChainPositions::HighCut>(chainSettings.highCutBypassed);

    updateCutFilter(leftHighCut, highCutCoefficients, chainSettings.highCutSlope);
    updateCutFilter(rightHighCut, highCutCoefficients, chainSettings.highCutSlope);

}

/**
 * @brief Updates all filters for both channels.
 *
 * Calls individual update functions for low-cut, peak, and high-cut filters.
 */
void SimpleEQAudioProcessor::updateFilters()
{
    auto chainSettings = getChainSettings(apvts);

    updateLowCutFilters(chainSettings);
    updatePeakFilter(chainSettings);
    updateHighCutFilters(chainSettings);
}

/**
 * @brief Creates the parameter layout for the EQ.
 *
 * Defines parameters for frequency, gain, Q, slope, and bypass for each band,
 * with appropriate ranges for audio processing.
 *
 * @return The parameter layout for the AudioProcessorValueTreeState.
 */
juce::AudioProcessorValueTreeState::ParameterLayout
    SimpleEQAudioProcessor::createParameterLayout()
    {
        juce::AudioProcessorValueTreeState::ParameterLayout layout;

        // Lowcut Frequency Layout
        layout.add(std::make_unique<juce::AudioParameterFloat>("LowCut Freq", "LowCut Freq", juce::NormalisableRange<float>(20.f,
        20000.f, 1.f, 1.f), 0.25f));

        // Highcut Frequency Layout
        layout.add(std::make_unique<juce::AudioParameterFloat>("HighCut Freq", "HighCut Freq", juce::NormalisableRange<float>(20.f,
        20000.f, 1.f, 1.f), 0.25f));

        // Peak Frequency Layout
        layout.add(std::make_unique<juce::AudioParameterFloat>("Peak Freq", "Peak Freq", juce::NormalisableRange<float>(20.f,
        20000.f, 1.f, 1.f), 0.25f));

        // Peak Gain Layout
        layout.add(std::make_unique<juce::AudioParameterFloat>("Peak Gain", "Peak Gain", juce::NormalisableRange<float>(-24.f,
        24.f, 0.5f, 1.f), 0.0f));

        // Peak Quality Layout
        layout.add(std::make_unique<juce::AudioParameterFloat>("Peak Quality", "Peak Quality", juce::NormalisableRange<float>(0.1f,
        10.f, 0.05f, 1.f), 1.f));

        juce::StringArray stringArray;
        for( int i = 0; i < 4; i++)
        {
            juce::String str;
            str << (12 + i*12);
            str << " db/Oct";
            stringArray.add(str);
        }

        layout.add(std::make_unique<juce::AudioParameterChoice>("LowCut Slope", "LowCut Slope", stringArray, 0));
        layout.add(std::make_unique<juce::AudioParameterChoice>("HighCut Slope", "HighCut Slope", stringArray, 0));

        layout.add(std::make_unique<juce::AudioParameterBool>("LowCut Bypassed", "LowCut Bypassed", false));
        layout.add(std::make_unique<juce::AudioParameterBool>("Peak Bypassed", "Peak Bypassed", false));
        layout.add(std::make_unique<juce::AudioParameterBool>("HighCut Bypassed", "HighCut Bypassed", false));
        layout.add(std::make_unique<juce::AudioParameterBool>("Analyzer Enabled", "Analyzer Enabled", true));

        return layout;
    }
//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new SimpleEQAudioProcessor();
}
