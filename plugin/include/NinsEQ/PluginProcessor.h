#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_dsp/juce_dsp.h>
#include <array>

namespace audio_plugin {

// Template FIFO implementation for audio buffer handling
template<typename T>
struct Fifo
{
    void prepare(int numChannels, int numSamples)
    {
        static_assert(std::is_same_v<T, juce::AudioBuffer<float>>,
                     "prepare(numChannels, numSamples) should only be used when the Fifo is holding juce::AudioBuffer<float>");
        for (auto& buffer : buffers)
        {
            buffer.setSize(numChannels,
                          numSamples,
                          false,   // clear everything?
                          true,    // including the extra space?
                          true);   // avoid reallocating if you can?
            buffer.clear();
        }
    }
    
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
    
    int getNumAvailableForReading() const
    {
        return fifo.getNumReady();
    }
    
private:
    static constexpr int Capacity = 30;
    std::array<T, Capacity> buffers;
    juce::AbstractFifo fifo{Capacity};
};

// Enum for channel selection
enum Channel
{
    Right, // effectively 0
    Left   // effectively 1
};

// Template for single channel sample FIFO
template<typename BlockType>
struct SingleChannelSampleFifo
{
    SingleChannelSampleFifo(Channel ch) : channelToUse(ch)
    {
        prepared.set(false);
    }
    
    void update(const BlockType& buffer)
    {
        jassert(prepared.get());
        jassert(buffer.getNumChannels() > channelToUse);
        auto* channelPtr = buffer.getReadPointer(channelToUse);
        
        for (int i = 0; i < buffer.getNumSamples(); ++i)
        {
            pushNextSampleIntoFifo(channelPtr[i]);
        }
    }

    void prepare(int bufferSize)
    {
        prepared.set(false);
        size.set(bufferSize);
        
        bufferToFill.setSize(1,             // channel
                            bufferSize,    // num samples
                            false,         // keepExistingContent
                            true,          // clear extra space
                            true);         // avoid reallocating
        audioBufferFifo.prepare(1, bufferSize);
        fifoIndex = 0;
        prepared.set(true);
    }
    
    int getNumCompleteBuffersAvailable() const { return audioBufferFifo.getNumAvailableForReading(); }
    bool isPrepared() const { return prepared.get(); }
    int getSize() const { return size.get(); }
    
    bool getAudioBuffer(BlockType& buf) { return audioBufferFifo.pull(buf); }
    
private:
    Channel channelToUse;
    int fifoIndex = 0;
    Fifo<BlockType> audioBufferFifo;
    BlockType bufferToFill;
    juce::Atomic<bool> prepared = false;
    juce::Atomic<int> size = 0;
    
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

// Parameter IDs for our plugin
namespace ParameterID {
    #define PARAMETER_ID(str) static const juce::ParameterID str(#str, 1);

    PARAMETER_ID(highpass_freq)
    PARAMETER_ID(highpass_q)
    
    PARAMETER_ID(mid_freq)
    PARAMETER_ID(mid_q)
    PARAMETER_ID(mid_gain)
    PARAMETER_ID(mid_threshold)
    PARAMETER_ID(mid_ratio)
    PARAMETER_ID(mid_attack)
    PARAMETER_ID(mid_release)
    
    PARAMETER_ID(lowpass_freq)
    PARAMETER_ID(lowpass_q)

    #undef PARAMETER_ID
}

class AudioPluginAudioProcessor : public juce::AudioProcessor {
public:
  AudioPluginAudioProcessor();
  ~AudioPluginAudioProcessor() override;

  void prepareToPlay(double sampleRate, int samplesPerBlock) override;
  void releaseResources() override;

  bool isBusesLayoutSupported(const BusesLayout& layouts) const override;

  void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
  using AudioProcessor::processBlock;

  juce::AudioProcessorEditor* createEditor() override;
  bool hasEditor() const override;

  const juce::String getName() const override;

  bool acceptsMidi() const override;
  bool producesMidi() const override;
  bool isMidiEffect() const override;
  double getTailLengthSeconds() const override;

  int getNumPrograms() override;
  int getCurrentProgram() override;
  void setCurrentProgram(int index) override;
  const juce::String getProgramName(int index) override;
  void changeProgramName(int index, const juce::String& newName) override;

  void getStateInformation(juce::MemoryBlock& destData) override;
  void setStateInformation(const void* data, int sizeInBytes) override;

  // FFT analysis methods and data
  void pushNextSampleIntoFifo(float sample, int channel, int fftChannel) noexcept;
  void drawNextFrameOfSpectrum();
  void drawFrame(juce::Graphics& g);
  
  // Function to access our parameters
  juce::AudioProcessorValueTreeState& getParameters() { return parameters; }
  
  // FFT data for the spectrum analyzer
  static constexpr auto fftOrder = 11;
  static constexpr auto fftSize = 1 << fftOrder;
  
  // Methods to access FFT data and gain reduction
  const float (*getScopeData() const)[fftSize] { return scopeData; }
  float getCurrentGainReduction() const { return currentGainReduction; }
  int getFFTSize() const { return fftSize; }
  
  float fifo[2][fftSize]; // One buffer per channel (stereo)
  float fftData[2][2 * fftSize];
  int fifoIndex = 0;
  bool nextFFTBlockReady = false;
  float scopeData[2][fftSize];
  
  // SingleChannelSampleFifo for better FFT data handling
  using BlockType = juce::AudioBuffer<float>;
  SingleChannelSampleFifo<BlockType> leftChannelFifo { Channel::Left };
  SingleChannelSampleFifo<BlockType> rightChannelFifo { Channel::Right };
  
private:
  // Create our filters
  using Filter = juce::dsp::IIR::Filter<float>;
  using MonoChain = juce::dsp::ProcessorChain<
      juce::dsp::ProcessorDuplicator<Filter, juce::dsp::IIR::Coefficients<float>>,
      juce::dsp::ProcessorDuplicator<Filter, juce::dsp::IIR::Coefficients<float>>,
      juce::dsp::ProcessorDuplicator<Filter, juce::dsp::IIR::Coefficients<float>>
  >;
  
  // Create one chain per channel (stereo)
  MonoChain leftChain, rightChain;
  
  // Enumeration for processor indices
  enum ChainPositions {
      HighPassFilter,
      MidBellFilter,
      LowPassFilter
  };
  
  // Level detector for dynamic EQ control
  juce::LinearSmoothedValue<float> midCompressorLevel;
  
  // Current gain reduction amount (for visualization)
  float currentGainReduction = 1.0f;
  
  // Parameter handling
  juce::AudioProcessorValueTreeState parameters;
  std::atomic<float>* highpassFreqParam = nullptr;
  std::atomic<float>* highpassQParam = nullptr;
  std::atomic<float>* midFreqParam = nullptr;
  std::atomic<float>* midQParam = nullptr;
  std::atomic<float>* midGainParam = nullptr;
  std::atomic<float>* midThresholdParam = nullptr;
  std::atomic<float>* midRatioParam = nullptr;
  std::atomic<float>* midAttackParam = nullptr;
  std::atomic<float>* midReleaseParam = nullptr;
  std::atomic<float>* lowpassFreqParam = nullptr;
  std::atomic<float>* lowpassQParam = nullptr;
  
  // FFT analyzer 
  juce::dsp::FFT forwardFFT;
  juce::dsp::WindowingFunction<float> window;
  
  // Update the filters based on parameters
  void updateFilters();
  
  JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(AudioPluginAudioProcessor)
};
}  // namespace audio_plugin
