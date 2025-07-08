/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

/**
 * @brief Enum defining FFT order options for audio analysis.
 */
enum FFTOrder
{
    order2048 = 11,
    order4096 = 12,
    order8192 = 13
};

/**
 * @brief Generates FFT data for real-time audio visualization.
 *
 * Processes audio buffers to produce frequency-domain data using a Fast Fourier Transform (FFT).
 * Applies a windowing function and normalizes the output for display in the GUI.
 *
 * @tparam BlockType The type of data stored (typically std::vector<float>).
 */
template<typename BlockType>
struct FFTDataGenerator
{
    /**
    Produces the FFT data from an audio buffer
    */
    void produceFFTDataForRendering(const juce::AudioBuffer<float>& audioData, const float negativeInfinity)
    {
        const auto fftSize = getFFTSize();

        fftData.assign(fftData.size(), 0);
        auto* readIndex = audioData.getReadPointer(0);
        std::copy(readIndex, readIndex + fftSize, fftData.begin());

        // First apply a windowing function for our data
        window->multiplyWithWindowingTable(fftData.data(), fftSize);

        // Then render our FFT data
        forwardFFT->performFrequencyOnlyForwardTransform(fftData.data());

        int numBins = (int)fftSize / 2;

        // Normalize the fft values
        for (int i = 0; i < numBins; i++)
        {
            fftData[i] /= (float)numBins;
        }

        // Convert them to decibels
        for (int i = 0; i < numBins; i++)
        {
            fftData[i] = juce::Decibels::gainToDecibels(fftData[i], negativeInfinity);
        }

        fftDataFifo.push(fftData);
    }

    /**
     * @brief Changes the FFT order and reinitializes resources.
     *
     * Updates the FFT size, recreates the window and FFT objects, and resets the FIFO.
     * Uses smart pointers for safe resource management.
     *
     * @param newOrder The new FFT order (e.g., order2048, order4096, order8192).
     */
    void changeOrder(FFTOrder newOrder)
    {
        /*
        When you change order, recreate the window, forwardFFT, fifo, fftData
        also reset the fifoIndex
        things that need recreating should be created on the heap via std::make_unique<>
        */

        order = newOrder;
        auto fftSize = getFFTSize();

        forwardFFT = std::make_unique<juce::dsp::FFT>(order);
        window = std::make_unique<juce::dsp::WindowingFunction<float>>(fftSize, juce::dsp::WindowingFunction<float>::blackmanHarris);

        fftData.clear();
        fftData.resize(fftSize * 2, 0);

        fftDataFifo.prepare(fftData.size());
    }
    //==============================================================================

    /**
     * @brief Returns the FFT size based on the current order.
     *
     * @return The FFT size (e.g., 2048, 4096, or 8192).
     */
    int getFFTSize() const { return 1 << order; }

    /**
     * @brief Returns the number of FFT data blocks available for reading.
     *
     * @return The number of blocks in the FIFO.
     */
    int getNumAvailableFFTDataBlocks() const { return fftDataFifo.getNumAvailableForReading(); }
    //==============================================================================

    /**
     * @brief Retrieves FFT data from the FIFO.
     *
     * @param fftData The vector to store the retrieved FFT data.
     * @return True if data was pulled, false if the FIFO is empty.
     */
    bool getFFTData(BlockType& fftData) 
    { 
        return fftDataFifo.pull(fftData);
        //return fftDataFifo.getNumAvailableForReading(); 
    }
private:
    FFTOrder order;
    BlockType fftData;
    std::unique_ptr<juce::dsp::FFT> forwardFFT;
    std::unique_ptr<juce::dsp::WindowingFunction<float>> window;

    Fifo<BlockType> fftDataFifo;
};


/**
 * @brief Converts FFT data into a juce::Path for visualization.
 *
 * Maps frequency-domain data to a graphical path for rendering the EQ response curve.
 *
 * @tparam PathType The type of path (typically juce::Path).
 */
template<typename PathType>
struct AnalyzerPathGenerator
{
    /**
     * @brief Generates a path from FFT data for rendering.
     *
     * Maps FFT bin amplitudes to a logarithmic frequency scale and creates a path
     * for visualization within the specified bounds.
     *
     * @param renderData The FFT data to convert.
     * @param fftBounds The rendering area bounds.
     * @param fftSize The FFT size.
     * @param binWidth The frequency width per bin (Hz).
     * @param negativeInfinity The minimum decibel value for mapping.
     */
    void genereatePath(
        const std::vector<float>& renderData,
        juce::Rectangle<float> fftBounds,
        int fftSize,
        float binWidth,
        float negativeInfinity)
    {
        auto top = fftBounds.getY();
        auto bottom = fftBounds.getHeight();
        auto width = fftBounds.getWidth();

        int numBins = (int)fftSize / 2;

        PathType p;
        p.preallocateSpace(3 * (int)fftBounds.getWidth());

        auto map = [bottom, top, negativeInfinity](float v)
        {
            return juce::jmap(
                v,
                negativeInfinity,
                0.f,
                float(bottom),
                top);
        };

        auto y = map(renderData[0]);

        jassert(!std::isnan(y) && !std::isinf(y));

        p.startNewSubPath(0, y);

        const int pathResolution = 2; // you can draw line-to's every 'pathResolution' pixels

        for (int binNum = 1; binNum < numBins; binNum += pathResolution)
        {
            y = map(renderData[binNum]);

            jassert(!std::isnan(y) && !std::isinf(y));

            if (!std::isnan(y) && !std::isinf(y))
            {
                auto binFreq = binNum * binWidth;
                auto normalizeBinX = juce::mapFromLog10(binFreq, 20.f, 20000.f);
                int binX = std::floor(normalizeBinX * width);
                p.lineTo(binX, y);
            }
        }

        pathFifo.push(p);
    }

    /**
     * @brief Returns the number of paths available for rendering.
     *
     * @return The number of paths in the FIFO.
     */
    int getNumPathsAvailable() const
    {
        return pathFifo.getNumAvailableForReading();
    }

    /**
     * @brief Retrieves a path from the FIFO.
     *
     * @param path The path to store the retrieved data.
     * @return True if a path was pulled, false if the FIFO is empty.
     */
    bool getPath(PathType& path)
    {
        return pathFifo.pull(path);
    }
private:
    Fifo<PathType> pathFifo;
};

/**
 * @brief Custom LookAndFeel for rotary sliders and toggle buttons.
 *
 * Overrides default JUCE styling for a consistent visual design.
 */
struct LookAndFeel : juce::LookAndFeel_V4
{
    /**
     * @brief Draws a rotary slider with custom styling.
     *
     * @param g The graphics context.
     * @param x The x-coordinate of the slider.
     * @param y The y-coordinate of the slider.
     * @param width The width of the slider.
     * @param height The height of the slider.
     * @param sliderPosProportional The normalized slider position (0 to 1).
     * @param rotaryStartAngle The start angle for the rotary arc.
     * @param rotaryEndAngle The end angle for the rotary arc.
     * @param slider The slider component.
     */
    void drawRotarySlider(juce::Graphics&,
        int x, int y, int width, int height,
        float sliderPosProportional,
        float rotaryStartAngle,
        float rotaryEndAngle,
        juce::Slider&) override;

    /**
     * @brief Draws a toggle button with custom styling.
     *
     * @param g The graphics context.
     * @param toggleButton The toggle button component.
     * @param shouldDrawButtonAsHighlighted True if the button is highlighted.
     * @param shouldDrawButtonAsDown True if the button is pressed.
     */
    void drawToggleButton(
        juce::Graphics& g,
        juce::ToggleButton& toggleButton,
        bool shouldDrawButtonAsHighlighted,
        bool shouldDrawButtonAsDown
    ) override;
};

/**
 * @brief A custom rotary slider with attached labels.
 *
 * Extends juce::Slider to display parameter values with units and custom styling.
 */
struct RotarySliderWithLabels : juce::Slider 
{
    /**
     * @brief Constructor.
     *
     * Initializes the slider with a rotary style and attaches it to a parameter.
     *
     * @param rap The audio parameter to control.
     * @param unitSuffix The unit suffix for display (e.g., "Hz", "dB").
     */
    RotarySliderWithLabels(juce::RangedAudioParameter& rap, const juce::String& unitSuffix) :
        juce::Slider(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag,
            juce::Slider::TextEntryBoxPosition::NoTextBox),
        param(&rap),
        suffix(unitSuffix)
    {
        setLookAndFeel(&lnf);
    }

    /**
     * @brief Destructor.
     *
     * Removes the custom LookAndFeel to prevent memory leaks.
     */
    ~RotarySliderWithLabels()
    {
        setLookAndFeel(nullptr);
    }

    /**
     * @brief Structure for label positions and text.
     */
    struct LabelPos
    {
        float pos;
        juce::String label;
    };

    juce::Array<LabelPos> labels;

    /**
     * @brief Paints the slider and its labels.
     *
     * @param g The graphics context.
     */
    void paint(juce::Graphics& g) override;

    /**
     * @brief Returns the bounds of the slider area.
     *
     * @return The slider's bounding rectangle.
     */
    juce::Rectangle<int> getSliderBounds() const;

    /**
     * @brief Returns the text height for labels.
     *
     * @return The height in pixels (default: 14).
     */
    int getTextHeight() const { return 14; }

    /**
     * @brief Returns the display string for the slider value.
     *
     * @return The parameter value with unit suffix.
     */
    juce::String getDisplayString() const;

private:
    LookAndFeel lnf;

    juce::RangedAudioParameter* param;
    juce::String suffix;
};

/**
 * @brief Generates FFT paths for visualization from a single-channel FIFO.
 */
struct PathProducer
{
    /**
     * @brief Constructor.
     *
     * Initializes the FFT data generator with a 2048-sample FFT and prepares a mono buffer.
     *
     * @param scsf The single-channel FIFO for audio data.
     */
    PathProducer(SingleChannelSampleFifo<SimpleEQAudioProcessor::BlockType>& scsf) :
        leftChannelFifo(&scsf)
    {
        leftChannelFFTDataGenerator.changeOrder(FFTOrder::order2048);
        monoBuffer.setSize(1, leftChannelFFTDataGenerator.getFFTSize());
    }

    /**
     * @brief Processes audio data to generate an FFT path.
     *
     * Converts FIFO audio data to an FFT path for rendering in the GUI.
     *
     * @param fftBounds The rendering area bounds.
     * @param sampleRate The audio sample rate.
     */
    void process(juce::Rectangle<float> fftBounds, double sampleRate);

    /**
     * @brief Returns the generated FFT path.
     *
     * @return The current FFT path for visualization.
     */
    juce::Path getPath() { return leftChannelFFTPath; }
private:
    SingleChannelSampleFifo<SimpleEQAudioProcessor::BlockType>* leftChannelFifo;

    juce::AudioBuffer<float> monoBuffer;

    FFTDataGenerator<std::vector<float>> leftChannelFFTDataGenerator;

    AnalyzerPathGenerator<juce::Path> pathProducer;

    juce::Path leftChannelFFTPath;
};

/**
 * @brief Component for rendering the EQ response curve and FFT analysis.
 *
 * Updates dynamically based on parameter changes and timer callbacks.
 */
struct ResponseCurveComponent : juce::Component,
    juce::AudioProcessorParameter::Listener,
    juce::Timer
{
    /**
     * @brief Constructor.
     *
     * Initializes the component with the audio processor and starts the timer.
     *
     * @param processor The SimpleEQAudioProcessor instance.
     */
    ResponseCurveComponent(SimpleEQAudioProcessor&);

    /**
     * @brief Destructor.
     *
     * Stops the timer and removes parameter listeners.
     */
    ~ResponseCurveComponent();

    /**
     * @brief Handles parameter value changes.
     *
     * Marks parameters as changed for updating the response curve.
     *
     * @param parameterIndex The index of the changed parameter.
     * @param newValue The new parameter value.
     */
    void parameterValueChanged(int parameterIndex, float newValue) override;
    void parameterGestureChanged(int parameterIndex, bool gestureIsStarting) override { }

    /**
     * @brief Handles timer callbacks to update the response curve.
     */
    void timerCallback() override;

    /**
     * @brief Paints the response curve and FFT visualization.
     *
     * @param g The graphics context.
     */
    void paint(juce::Graphics& g) override;

    /**
     * @brief Lays out the component when resized.
     */
    void resized() override;

    /**
     * @brief Toggles FFT analysis rendering.
     *
     * @param enabled True to enable FFT visualization, false to disable.
     */
    void toggleAnalysisEnablement(bool enabled)
    {
        shouldShowFFTAnalysis = enabled;
    }
private:
    SimpleEQAudioProcessor& audioProcessor;
    juce::Atomic<bool> parametersChanged{ false };

    MonoChain monoChain;

    void updateChain();

    juce::Image background;

    juce::Rectangle<int> getRenderArea();

    juce::Rectangle<int> getAnalysisArea();

    PathProducer leftPathProducer, rightPathProducer;

    bool shouldShowFFTAnalysis = true;
};

//==============================================================================
/**
 * @brief A toggle button for bypass controls.
 */
struct PowerButton : juce::ToggleButton { };

/**
 * @brief A toggle button for enabling/disabling the analyzer with a random path.
 */
struct AnalyzerButton : juce::ToggleButton 
{
    /**
     * @brief Updates the random path when the button is resized.
     */
    void resized() override
    {
        auto bounds = getLocalBounds();
        auto insetRect = bounds.reduced(4);

        randomPath.clear();

        juce::Random r;

        // Start random path at a random height
        randomPath.startNewSubPath(insetRect.getX(), insetRect.getY() + insetRect.getHeight() * r.nextFloat());

        // Generate random path points
        for (auto x = insetRect.getX() + 1; x < insetRect.getRight(); x += 2)
        {
            randomPath.lineTo(x, insetRect.getY() + insetRect.getHeight() * r.nextFloat());
        }
    }

    juce::Path randomPath;
};

/**
 * @brief The editor for the SimpleEQ plugin.
 *
 * Provides a GUI with sliders for EQ parameters, toggle buttons for bypass,
 * and a response curve component with FFT visualization.
 */
class SimpleEQAudioProcessorEditor  : public juce::AudioProcessorEditor
{
public:
    /**
     * @brief Constructor.
     *
     * Initializes sliders, buttons, and the response curve component, linking them
     * to the processor's parameters.
     *
     * @param processor The SimpleEQAudioProcessor instance.
     */
    SimpleEQAudioProcessorEditor (SimpleEQAudioProcessor&);

    /**
     * @brief Destructor.
     *
     * Cleans up components and parameter attachments.
     */
    ~SimpleEQAudioProcessorEditor() override;

    //==============================================================================

    /**
     * @brief Paints the editor's background and static elements.
     *
     * @param g The graphics context.
     */
    void paint (juce::Graphics&) override;

    /**
     * @brief Lays out sliders, buttons, and the response curve component.
     */
    void resized() override;

private:
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    SimpleEQAudioProcessor& audioProcessor;

    // Sliders for EQ parameters
    RotarySliderWithLabels peakFreqSlider,
        peakGainSlider,
        peakQualitySlider,
        lowCutFreqSlider,
        highCutFreqSlider,
        lowCutSlopeSlider,
        highCutSlopeSlider;

    ResponseCurveComponent responseCurveComponent;

    using APVTS = juce::AudioProcessorValueTreeState;
    using Attachment = APVTS::SliderAttachment;

    // Slider attachments for parameter linking
    Attachment peakFreqSliderAttachment,
        peakGainSliderAttachment,
        peakQualitySliderAttachment,
        lowCutFreqSliderAttachment,
        highCutFreqSliderAttachment,
        lowCutSlopeSliderAttachment,
        highCutSlopeSliderAttachment;

    // Toggle buttons for bypass and analyzer
    PowerButton lowcutBypassButton, peakBypassButton, highcutBypassButton;
    AnalyzerButton analyzerEnabledButton;

    // Button attachments for parameter linking
    using ButtonAttachment = APVTS::ButtonAttachment;
    ButtonAttachment lowcutBypassButtonAttachment, 
                     peakBypassButtonAttachment,
                     highcutBypassButtonAttachment,
                     analyzerEnabledButtonAttachment;

    /**
     * @brief Returns a list of all GUI components.
     *
     * @return A vector of pointers to components (sliders, buttons, etc.).
     */
    std::vector<juce::Component*> getComps();

    LookAndFeel lnf;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SimpleEQAudioProcessorEditor)
};
