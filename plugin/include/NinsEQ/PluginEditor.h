#pragma once

#include "PluginProcessor.h"

namespace audio_plugin {

// FFT order enumeration
enum FFTOrder
{
    order2048 = 11,
    order4096 = 12,
    order8192 = 13
};

// FFT data generator for processing audio data into frequency spectrum
template<typename BlockType>
struct FFTDataGenerator
{
    /**
     * Produces the FFT data from an audio buffer.
     */
    void produceFFTDataForRendering(const juce::AudioBuffer<float>& audioData, const float negativeInfinity)
    {
        const auto fftSize = getFFTSize();
        
        fftData.assign(fftData.size(), 0);
        auto* readIndex = audioData.getReadPointer(0);
        std::copy(readIndex, readIndex + fftSize, fftData.begin());
        
        // First apply a windowing function to our data
        window->multiplyWithWindowingTable(fftData.data(), fftSize);
        
        // Then perform the FFT
        forwardFFT->performFrequencyOnlyForwardTransform(fftData.data());
        
        int numBins = (int)fftSize / 2;
        
        // Normalize the FFT values
        for (int i = 0; i < numBins; ++i)
        {
            auto v = fftData[i];
            if (!std::isinf(v) && !std::isnan(v))
            {
                v /= float(numBins);
            }
            else
            {
                v = 0.f;
            }
            fftData[i] = v;
        }
        
        // Convert to decibels
        for (int i = 0; i < numBins; ++i)
        {
            fftData[i] = juce::Decibels::gainToDecibels(fftData[i], negativeInfinity);
        }
        
        fftDataFifo.push(fftData);
    }
    
    void changeOrder(FFTOrder newOrder)
    {
        // When you change order, recreate everything
        order = newOrder;
        auto fftSize = getFFTSize();
        
        forwardFFT = std::make_unique<juce::dsp::FFT>(order);
        window = std::make_unique<juce::dsp::WindowingFunction<float>>(fftSize, juce::dsp::WindowingFunction<float>::blackmanHarris);
        
        fftData.clear();
        fftData.resize(fftSize * 2, 0);

        fftDataFifo.prepare(fftData.size());
    }
    
    int getFFTSize() const { return 1 << order; }
    int getNumAvailableFFTDataBlocks() const { return fftDataFifo.getNumAvailableForReading(); }
    bool getFFTData(BlockType& fftData) { return fftDataFifo.pull(fftData); }
    
private:
    FFTOrder order;
    BlockType fftData;
    std::unique_ptr<juce::dsp::FFT> forwardFFT;
    std::unique_ptr<juce::dsp::WindowingFunction<float>> window;
    
    Fifo<BlockType> fftDataFifo;
};

// Path generator for converting FFT data to display paths
template<typename PathType>
struct AnalyzerPathGenerator
{
    /*
     * Converts renderData into a juce::Path for visualization
     */
    void generatePath(const std::vector<float>& renderData,
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
            return juce::jmap(v,
                             negativeInfinity, 0.f,
                             float(bottom+10), top);
        };

        auto y = map(renderData[0]);
        
        if (std::isnan(y) || std::isinf(y))
            y = bottom;
        
        p.startNewSubPath(0, y);

        const int pathResolution = 2; // Draw line-to's every 'pathResolution' pixels

        for (int binNum = 1; binNum < numBins; binNum += pathResolution)
        {
            y = map(renderData[binNum]);

            if (!std::isnan(y) && !std::isinf(y))
            {
                auto binFreq = binNum * binWidth;
                auto normalizedBinX = juce::mapFromLog10(binFreq, 20.f, 20000.f);
                int binX = std::floor(normalizedBinX * width);
                p.lineTo(binX, y);
            }
        }

        pathFifo.push(p);
    }

    int getNumPathsAvailable() const
    {
        return pathFifo.getNumAvailableForReading();
    }

    bool getPath(PathType& path)
    {
        return pathFifo.pull(path);
    }
    
private:
    Fifo<PathType> pathFifo;
};

// Path producer to manage processing of FFT data into paths
struct PathProducer
{
    PathProducer(SingleChannelSampleFifo<AudioPluginAudioProcessor::BlockType>& scsf) :
    channelFifo(&scsf)
    {
        fftDataGenerator.changeOrder(FFTOrder::order2048);
        monoBuffer.setSize(1, fftDataGenerator.getFFTSize());
    }
    
    void process(juce::Rectangle<float> fftBounds, double sampleRate);
    juce::Path getPath() { return channelFFTPath; }
    
private:
    SingleChannelSampleFifo<AudioPluginAudioProcessor::BlockType>* channelFifo;
    
    juce::AudioBuffer<float> monoBuffer;
    
    FFTDataGenerator<std::vector<float>> fftDataGenerator;
    
    AnalyzerPathGenerator<juce::Path> pathProducer;
    
    juce::Path channelFFTPath;
};

class FrequencyResponseCurveComponent : public juce::Component,
                                       private juce::Timer
{
public:
    FrequencyResponseCurveComponent(AudioPluginAudioProcessor&);
    ~FrequencyResponseCurveComponent() override;
    
    void paint(juce::Graphics&) override;
    void resized() override;
    
    void updateCurve();
    
private:
    AudioPluginAudioProcessor& audioProcessor;
    
    juce::Path responseCurve;
    
    void timerCallback() override;
    
    // Draw the filter response curve separately
    void drawFilterResponse(juce::Graphics& g, const juce::Rectangle<int>& bounds);
    
    // Helper methods for coordinate mapping
    double mapFrequencyToX(double freq, int width) const;
    double mapDbToY(double db, int height) const;
    
    // Path producers for left and right channels
    PathProducer leftPathProducer, rightPathProducer;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(FrequencyResponseCurveComponent)
};

class AudioPluginAudioProcessorEditor : public juce::AudioProcessorEditor,
                                       private juce::Timer
{
public:
    explicit AudioPluginAudioProcessorEditor(AudioPluginAudioProcessor&);
    ~AudioPluginAudioProcessorEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    // Reference to the processor object
    AudioPluginAudioProcessor& processorRef;
    
    // UI Components
    juce::Slider highpassFreqSlider;
    juce::Slider highpassQSlider;
    juce::Slider midFreqSlider;
    juce::Slider midQSlider;
    juce::Slider midGainSlider;
    juce::Slider midThresholdSlider;
    juce::Slider midRatioSlider;
    juce::Slider midAttackSlider;
    juce::Slider midReleaseSlider;
    juce::Slider lowpassFreqSlider;
    juce::Slider lowpassQSlider;
    
    // Slider Labels
    juce::Label highpassFreqLabel;
    juce::Label highpassQLabel;
    juce::Label midFreqLabel;
    juce::Label midQLabel;
    juce::Label midGainLabel;
    juce::Label midThresholdLabel;
    juce::Label midRatioLabel;
    juce::Label midAttackLabel;
    juce::Label midReleaseLabel;
    juce::Label lowpassFreqLabel;
    juce::Label lowpassQLabel;
    
    // Slider attachments to connect to parameters
    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    std::unique_ptr<SliderAttachment> highpassFreqAttachment;
    std::unique_ptr<SliderAttachment> highpassQAttachment;
    std::unique_ptr<SliderAttachment> midFreqAttachment;
    std::unique_ptr<SliderAttachment> midQAttachment;
    std::unique_ptr<SliderAttachment> midGainAttachment;
    std::unique_ptr<SliderAttachment> midThresholdAttachment;
    std::unique_ptr<SliderAttachment> midRatioAttachment;
    std::unique_ptr<SliderAttachment> midAttackAttachment;
    std::unique_ptr<SliderAttachment> midReleaseAttachment;
    std::unique_ptr<SliderAttachment> lowpassFreqAttachment;
    std::unique_ptr<SliderAttachment> lowpassQAttachment;
    
    // Custom components
    FrequencyResponseCurveComponent responseCurveComponent;
    
    // Update timer callback for spectrum analyzer
    void timerCallback() override;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(AudioPluginAudioProcessorEditor)
};
}  // namespace audio_plugin
