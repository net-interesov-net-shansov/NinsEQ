#include "NinsEQ/PluginProcessor.h"
#include "NinsEQ/PluginEditor.h"

namespace audio_plugin {

AudioPluginAudioProcessor::AudioPluginAudioProcessor()
    : AudioProcessor(
          BusesProperties()
#if !JucePlugin_IsMidiEffect
#if !JucePlugin_IsSynth
              .withInput("Input", juce::AudioChannelSet::stereo(), true)
#endif
              .withOutput("Output", juce::AudioChannelSet::stereo(), true)
#endif
      ),
      forwardFFT(fftOrder),
      window(fftSize, juce::dsp::WindowingFunction<float>::hann),
      parameters(*this, nullptr, "Parameters", {
          // Highpass filter parameters
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::highpass_freq, "Highpass Frequency",
              juce::NormalisableRange<float>(20.0f, 20000.0f, 1.0f, 0.25f),
              80.0f, "Hz"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::highpass_q, "Highpass Q",
              juce::NormalisableRange<float>(0.1f, 10.0f, 0.01f, 0.5f),
              0.707f),
          
          // Mid band parameters
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_freq, "Mid Frequency",
              juce::NormalisableRange<float>(20.0f, 20000.0f, 1.0f, 0.25f),
              1000.0f, "Hz"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_q, "Mid Q",
              juce::NormalisableRange<float>(0.1f, 10.0f, 0.01f, 0.5f),
              1.0f),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_gain, "Mid Gain",
              juce::NormalisableRange<float>(-24.0f, 24.0f, 0.1f, 1.0f),
              0.0f, "dB"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_threshold, "Mid Threshold",
              juce::NormalisableRange<float>(-60.0f, 12.0f, 0.1f, 1.0f),
              -12.0f, "dB"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_ratio, "Mid Ratio",
              juce::NormalisableRange<float>(1.0f, 20.0f, 0.1f, 0.5f),
              4.0f, ":1"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_attack, "Mid Attack",
              juce::NormalisableRange<float>(0.1f, 200.0f, 0.1f, 0.5f),
              50.0f, "ms"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::mid_release, "Mid Release",
              juce::NormalisableRange<float>(5.0f, 5000.0f, 1.0f, 0.3f),
              200.0f, "ms"),
          
          // Lowpass filter parameters
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::lowpass_freq, "Lowpass Frequency",
              juce::NormalisableRange<float>(20.0f, 20000.0f, 1.0f, 0.25f),
              12000.0f, "Hz"),
          std::make_unique<juce::AudioParameterFloat>(
              ParameterID::lowpass_q, "Lowpass Q",
              juce::NormalisableRange<float>(0.1f, 10.0f, 0.01f, 0.5f),
              0.707f)
      })
{
    // Initialize the fifo buffers
    for (int channel = 0; channel < 2; ++channel)
    {
        std::fill(fifo[channel], fifo[channel] + fftSize, 0.0f);
        std::fill(fftData[channel], fftData[channel] + 2 * fftSize, 0.0f);
        std::fill(scopeData[channel], scopeData[channel] + fftSize, 0.0f);
    }
    
    // Get parameter pointers
    highpassFreqParam = parameters.getRawParameterValue(ParameterID::highpass_freq.getParamID());
    highpassQParam = parameters.getRawParameterValue(ParameterID::highpass_q.getParamID());
    midFreqParam = parameters.getRawParameterValue(ParameterID::mid_freq.getParamID());
    midQParam = parameters.getRawParameterValue(ParameterID::mid_q.getParamID());
    midGainParam = parameters.getRawParameterValue(ParameterID::mid_gain.getParamID());
    midThresholdParam = parameters.getRawParameterValue(ParameterID::mid_threshold.getParamID());
    midRatioParam = parameters.getRawParameterValue(ParameterID::mid_ratio.getParamID());
    midAttackParam = parameters.getRawParameterValue(ParameterID::mid_attack.getParamID());
    midReleaseParam = parameters.getRawParameterValue(ParameterID::mid_release.getParamID());
    lowpassFreqParam = parameters.getRawParameterValue(ParameterID::lowpass_freq.getParamID());
    lowpassQParam = parameters.getRawParameterValue(ParameterID::lowpass_q.getParamID());
}

AudioPluginAudioProcessor::~AudioPluginAudioProcessor() {}

const juce::String AudioPluginAudioProcessor::getName() const {
  return JucePlugin_Name;
}

bool AudioPluginAudioProcessor::acceptsMidi() const {
#if JucePlugin_WantsMidiInput
  return true;
#else
  return false;
#endif
}

bool AudioPluginAudioProcessor::producesMidi() const {
#if JucePlugin_ProducesMidiOutput
  return true;
#else
  return false;
#endif
}

bool AudioPluginAudioProcessor::isMidiEffect() const {
#if JucePlugin_IsMidiEffect
  return true;
#else
  return false;
#endif
}

double AudioPluginAudioProcessor::getTailLengthSeconds() const {
  return 0.0;
}

int AudioPluginAudioProcessor::getNumPrograms() {
  return 1;  // NB: some hosts don't cope very well if you tell them there are 0
             // programs, so this should be at least 1, even if you're not
             // really implementing programs.
}

int AudioPluginAudioProcessor::getCurrentProgram() {
  return 0;
}

void AudioPluginAudioProcessor::setCurrentProgram(int index) {
  juce::ignoreUnused(index);
}

const juce::String AudioPluginAudioProcessor::getProgramName(int index) {
  juce::ignoreUnused(index);
  return {};
}

void AudioPluginAudioProcessor::changeProgramName(int index,
                                                  const juce::String& newName) {
  juce::ignoreUnused(index, newName);
}

void AudioPluginAudioProcessor::prepareToPlay(double sampleRate,
                                              int samplesPerBlock) {
    // Prepare DSP objects for playback
    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = samplesPerBlock;
    spec.numChannels = getTotalNumOutputChannels();
    
    leftChain.prepare(spec);
    rightChain.prepare(spec);
    
    // Initialize the compressor level detector
    midCompressorLevel.reset(sampleRate, 0.05);
    
    // Reset the fifo
    fifoIndex = 0;
    nextFFTBlockReady = false;
    
    // Prepare the FFT FIFOs
    leftChannelFifo.prepare(samplesPerBlock);
    rightChannelFifo.prepare(samplesPerBlock);
    
    // Update filter coefficients
    updateFilters();
}

void AudioPluginAudioProcessor::updateFilters()
{
    // Get current parameter values
    float highpassFreq = highpassFreqParam->load();
    float highpassQ = highpassQParam->load();
    float midFreq = midFreqParam->load();
    float midQ = midQParam->load();
    float midGain = midGainParam->load();
    float lowpassFreq = lowpassFreqParam->load();
    float lowpassQ = lowpassQParam->load();
    
    // Update highpass filter
    *leftChain.get<HighPassFilter>().state = *juce::dsp::IIR::Coefficients<float>::makeHighPass(
        getSampleRate(), highpassFreq, highpassQ);
    *rightChain.get<HighPassFilter>().state = *juce::dsp::IIR::Coefficients<float>::makeHighPass(
        getSampleRate(), highpassFreq, highpassQ);
    
    // Update bell filter (mid band)
    *leftChain.get<MidBellFilter>().state = *juce::dsp::IIR::Coefficients<float>::makePeakFilter(
        getSampleRate(), midFreq, midQ, juce::Decibels::decibelsToGain(midGain));
    *rightChain.get<MidBellFilter>().state = *juce::dsp::IIR::Coefficients<float>::makePeakFilter(
        getSampleRate(), midFreq, midQ, juce::Decibels::decibelsToGain(midGain));
    
    // Update lowpass filter
    *leftChain.get<LowPassFilter>().state = *juce::dsp::IIR::Coefficients<float>::makeLowPass(
        getSampleRate(), lowpassFreq, lowpassQ);
    *rightChain.get<LowPassFilter>().state = *juce::dsp::IIR::Coefficients<float>::makeLowPass(
        getSampleRate(), lowpassFreq, lowpassQ);
}

void AudioPluginAudioProcessor::releaseResources() {
  // When playback stops, you can use this as an opportunity to free up any
  // spare memory, etc.
}

bool AudioPluginAudioProcessor::isBusesLayoutSupported(
    const BusesLayout& layouts) const {
#if JucePlugin_IsMidiEffect
  juce::ignoreUnused(layouts);
  return true;
#else
  // This is the place where you check if the layout is supported.
  // In this template code we only support mono or stereo.
  // Some plugin hosts, such as certain GarageBand versions, will only
  // load plugins that support stereo bus layouts.
  if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono() &&
      layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
    return false;

  // This checks if the input layout matches the output layout
#if !JucePlugin_IsSynth
  if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
    return false;
#endif

  return true;
#endif
}

void AudioPluginAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer,
                                             juce::MidiBuffer& midiMessages) {
    juce::ignoreUnused(midiMessages);

    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // Clear output channels that don't have input data
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    // Update filters with current parameter values
    updateFilters();
    
    // Calculate dynamic gain for mid band filter based on input level
    float threshold = juce::Decibels::decibelsToGain(midThresholdParam->load());
    float ratio = midRatioParam->load();
    float attack = midAttackParam->load() / 1000.0f; // convert ms to seconds
    float release = midReleaseParam->load() / 1000.0f; // convert ms to seconds
    
    // Configure the level smoothing
    midCompressorLevel.reset(getSampleRate(), attack);
    midCompressorLevel.setCurrentAndTargetValue(0.0f); // Use release time for decay
    
    // Make a copy of the input buffer for FFT analysis (before processing)
    juce::AudioBuffer<float> inputCopy;
    inputCopy.makeCopyOf(buffer);
    
    // Feed input buffer copy to left channel FIFO for spectrum analysis
    if (leftChannelFifo.isPrepared())
    {
        leftChannelFifo.update(inputCopy);
    }
    
    // Get audio block for processing
    juce::dsp::AudioBlock<float> block(buffer);
    
    // Process left and right channels separately
    if (totalNumInputChannels >= 2) {
        // Split into left and right channels
        auto leftBlock = block.getSingleChannelBlock(0);
        auto rightBlock = block.getSingleChannelBlock(1);
        
        juce::dsp::ProcessContextReplacing<float> leftContext(leftBlock);
        juce::dsp::ProcessContextReplacing<float> rightContext(rightBlock);
        
        // Compute RMS level for dynamic processing
        float inputLevel = 0.0f;
        for (int channel = 0; channel < totalNumInputChannels; ++channel) {
            auto channelRMS = buffer.getRMSLevel(channel, 0, buffer.getNumSamples());
            inputLevel = std::max(inputLevel, channelRMS);
        }
        
        // Smooth the level for more natural dynamic response
        midCompressorLevel.setTargetValue(inputLevel);
        
        // Compute the gain reduction
        float smoothedLevel = midCompressorLevel.getNextValue();
        float gainReduction = 1.0f;
        
        if (smoothedLevel > threshold) {
            // Compute compression amount
            float dbAboveThreshold = juce::Decibels::gainToDecibels(smoothedLevel / threshold);
            float compressedDb = dbAboveThreshold / ratio;
            float dbGainReduction = compressedDb - dbAboveThreshold;
            
            // Convert back to linear gain
            gainReduction = juce::Decibels::decibelsToGain(dbGainReduction);
        }
        
        // Store the current gain reduction for visualization
        currentGainReduction = gainReduction;
        
        // Update mid band filter gain with dynamic processing
        float staticGain = juce::Decibels::decibelsToGain(midGainParam->load());
        float dynamicGain = staticGain * gainReduction;
        
        // Update mid band filter with dynamic gain
        *leftChain.get<MidBellFilter>().state = *juce::dsp::IIR::Coefficients<float>::makePeakFilter(
            getSampleRate(), midFreqParam->load(), midQParam->load(), dynamicGain);
        *rightChain.get<MidBellFilter>().state = *juce::dsp::IIR::Coefficients<float>::makePeakFilter(
            getSampleRate(), midFreqParam->load(), midQParam->load(), dynamicGain);
            
        // Process through filter chains
        leftChain.process(leftContext);
        rightChain.process(rightContext);
    } else if (totalNumInputChannels == 1) {
        // Mono processing
        auto monoBlock = block.getSingleChannelBlock(0);
        juce::dsp::ProcessContextReplacing<float> monoContext(monoBlock);
        
        // Compute RMS level for dynamic processing
        float inputLevel = buffer.getRMSLevel(0, 0, buffer.getNumSamples());
        
        // Smooth the level for more natural dynamic response
        midCompressorLevel.setTargetValue(inputLevel);
        
        // Compute the gain reduction
        float smoothedLevel = midCompressorLevel.getNextValue();
        float gainReduction = 1.0f;
        
        if (smoothedLevel > threshold) {
            // Compute compression amount
            float dbAboveThreshold = juce::Decibels::gainToDecibels(smoothedLevel / threshold);
            float compressedDb = dbAboveThreshold / ratio;
            float dbGainReduction = compressedDb - dbAboveThreshold;
            
            // Convert back to linear gain
            gainReduction = juce::Decibels::decibelsToGain(dbGainReduction);
        }
        
        // Store the current gain reduction for visualization
        currentGainReduction = gainReduction;
        
        // Update mid band filter gain with dynamic processing
        float staticGain = juce::Decibels::decibelsToGain(midGainParam->load());
        float dynamicGain = staticGain * gainReduction;
        
        // Update mid band filter with dynamic gain
        *leftChain.get<MidBellFilter>().state = *juce::dsp::IIR::Coefficients<float>::makePeakFilter(
            getSampleRate(), midFreqParam->load(), midQParam->load(), dynamicGain);
        
        // Process through filter chain
        leftChain.process(monoContext);
    }
    
    // Feed processed buffer to right channel FIFO for spectrum analysis
    if (rightChannelFifo.isPrepared())
    {
        rightChannelFifo.update(buffer);
    }
    
    // Legacy FFT code - we're using the new FIFO system above instead
    // Push original (unprocessed) samples into FFT fifo for input spectrum analysis (from input copy)
    for (int channel = 0; channel < juce::jmin(2, totalNumInputChannels); ++channel) {
        auto* channelData = inputCopy.getReadPointer(channel);
        
        for (int sample = 0; sample < inputCopy.getNumSamples(); ++sample) {
            pushNextSampleIntoFifo(channelData[sample], channel, 0); // Channel 0 of FFT data for input
        }
    }
    
    // Push processed samples into FFT fifo for output spectrum analysis
    for (int channel = 0; channel < juce::jmin(2, totalNumInputChannels); ++channel) {
        auto* channelData = buffer.getReadPointer(channel);
        
        for (int sample = 0; sample < buffer.getNumSamples(); ++sample) {
            pushNextSampleIntoFifo(channelData[sample], channel, 1); // Channel 1 of FFT data for output
        }
    }
}

void AudioPluginAudioProcessor::pushNextSampleIntoFifo(float sample, int channel, int fftChannel) noexcept
{
    if (fifoIndex == fftSize) {
        if (!nextFFTBlockReady) {
            // Copy data from fifo to fftData for FFT processing
            std::copy(fifo[channel], fifo[channel] + fftSize, fftData[fftChannel]);
            nextFFTBlockReady = true;
        }
        
        fifoIndex = 0;
    }
    
    fifo[channel][fifoIndex] = sample;
    ++fifoIndex;
}

void AudioPluginAudioProcessor::drawNextFrameOfSpectrum()
{
    // Return if no new FFT data available
    if (!nextFFTBlockReady)
        return;
    
    // Process FFT for both input and output channels
    for (int fftChannel = 0; fftChannel < 2; ++fftChannel) {
        // Apply windowing function to data
        window.multiplyWithWindowingTable(fftData[fftChannel], fftSize);
        
        // Perform FFT
        forwardFFT.performFrequencyOnlyForwardTransform(fftData[fftChannel]);
        
        // Convert to decibels and copy to scope data
        for (int i = 0; i < fftSize / 2; ++i) {
            // Scale for display (adjust as needed for visualization)
            auto skewedProportionX = 1.0f - std::exp(std::log(1.0f - (float)i / (float)(fftSize / 2)) * 0.2f);
            auto fftDataIndex = juce::jlimit(0, fftSize / 2, (int)(skewedProportionX * (float)fftSize / 2));
            
            // Convert to decibels with range limiting
            auto level = juce::jlimit(-100.0f, 0.0f, juce::Decibels::gainToDecibels(fftData[fftChannel][fftDataIndex])
                                      - juce::Decibels::gainToDecibels((float)fftSize));
            
            // Normalize to 0.0-1.0 range for display
            scopeData[fftChannel][i] = juce::jmap(level, -100.0f, 0.0f, 0.0f, 1.0f);
        }
    }
    
    nextFFTBlockReady = false;
    
    // Update the FFT data before each frame
    forwardFFT.performFrequencyOnlyForwardTransform(fftData[0]);
    forwardFFT.performFrequencyOnlyForwardTransform(fftData[1]);
}

void AudioPluginAudioProcessor::drawFrame(juce::Graphics& g)
{
    // Call before drawing to update spectrum data
    drawNextFrameOfSpectrum();
    
    // Get drawing area
    auto width = g.getClipBounds().getWidth();
    auto height = g.getClipBounds().getHeight();
    
    // Draw spectrum for each channel
    for (int channel = 0; channel < 2; ++channel) {
        // Choose color based on channel
        g.setColour(channel == 0 ? juce::Colours::orangered : juce::Colours::yellowgreen);
        
        // Draw FFT spectrum
        for (int i = 1; i < fftSize / 2; ++i) {
            // Map FFT bin frequencies to x coordinates using logarithmic scaling
            auto binFrequency = (double)i * getSampleRate() / (double)fftSize;
            auto normalizedBinX = std::log10(binFrequency / 20.0) / std::log10(20000.0 / 20.0);
            auto binX = normalizedBinX * width;
            
            // Draw line from previous point
            if (i > 1) {
                auto prevBinFrequency = (double)(i - 1) * getSampleRate() / (double)fftSize;
                auto prevNormalizedBinX = std::log10(prevBinFrequency / 20.0) / std::log10(20000.0 / 20.0);
                auto prevBinX = prevNormalizedBinX * width;
                
                g.drawLine(prevBinX, 
                          height * (1.0f - scopeData[channel][i - 1]),
                          binX,
                          height * (1.0f - scopeData[channel][i]),
                          1.0f);
            }
        }
  }
}

bool AudioPluginAudioProcessor::hasEditor() const {
  return true;  // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* AudioPluginAudioProcessor::createEditor() {
  return new AudioPluginAudioProcessorEditor(*this);
}

void AudioPluginAudioProcessor::getStateInformation(
    juce::MemoryBlock& destData) {
    // Save plugin state
    auto state = parameters.copyState();
    std::unique_ptr<juce::XmlElement> xml(state.createXml());
    copyXmlToBinary(*xml, destData);
}

void AudioPluginAudioProcessor::setStateInformation(const void* data,
                                                    int sizeInBytes) {
    // Restore plugin state
    std::unique_ptr<juce::XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes));
    
    if (xmlState.get() != nullptr && xmlState->hasTagName(parameters.state.getType())) {
        parameters.replaceState(juce::ValueTree::fromXml(*xmlState));
    }
}
}  // namespace audio_plugin

// This creates new instances of the plugin.
// This function definition must be in the global namespace.
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() {
  return new audio_plugin::AudioPluginAudioProcessor();
}
