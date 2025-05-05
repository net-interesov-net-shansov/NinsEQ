#include <NinsEQ/PluginProcessor.h>
#include <gtest/gtest.h>
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>
#include <cmath>

namespace audio_plugin_test {

class AudioProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Common setup for all tests
        processor.reset(new audio_plugin::AudioPluginAudioProcessor());
        sampleRate = 44100.0;
        blockSize = 512;
        processor->prepareToPlay(sampleRate, blockSize);
    }

    void TearDown() override {
        processor->releaseResources();
        processor.reset();
    }

    // Helper method to create and fill a test buffer
    juce::AudioBuffer<float> createTestBuffer(int numChannels, int numSamples, float frequency = 1000.0f) {
        juce::AudioBuffer<float> buffer(numChannels, numSamples);
        
        // Fill with sine wave
        for (int channel = 0; channel < numChannels; ++channel) {
            auto* channelData = buffer.getWritePointer(channel);
            
            for (int sample = 0; sample < numSamples; ++sample) {
                channelData[sample] = static_cast<float>(std::sin(2.0 * juce::MathConstants<double>::pi * frequency * sample / sampleRate));
            }
        }
        
        return buffer;
    }

    // Helper to check if a buffer contains NaN values
    bool containsNaN(const juce::AudioBuffer<float>& buffer, int channel) {
        auto* data = buffer.getReadPointer(channel);
        for (int i = 0; i < buffer.getNumSamples(); ++i) {
            if (std::isnan(data[i])) return true;
        }
        return false;
    }

    std::unique_ptr<audio_plugin::AudioPluginAudioProcessor> processor;
    double sampleRate;
    int blockSize;
};

// Test that the plugin initializes with expected default values
TEST_F(AudioProcessorTest, Initialization) {
    // Verify the plugin initializes correctly
    EXPECT_EQ(processor->getName(), "NinsEQ");
    EXPECT_FALSE(processor->acceptsMidi());
    EXPECT_FALSE(processor->producesMidi());
    EXPECT_FALSE(processor->isMidiEffect());
    
    // Test basic bus layout support
    juce::AudioProcessor::BusesLayout stereoLayout;
    stereoLayout.inputBuses.add(juce::AudioChannelSet::stereo());
    stereoLayout.outputBuses.add(juce::AudioChannelSet::stereo());
    EXPECT_TRUE(processor->isBusesLayoutSupported(stereoLayout));
}

// Test that parameters exist and are within range
TEST_F(AudioProcessorTest, ParameterInitialization) {
    // Get reference to parameters
    auto& params = processor->getParameters();
    
    // Check if all parameters exist and have correct defaults
    auto highpassFreq = params.getParameter(audio_plugin::ParameterID::highpass_freq.getParamID());
    auto midFreq = params.getParameter(audio_plugin::ParameterID::mid_freq.getParamID());
    auto lowpassFreq = params.getParameter(audio_plugin::ParameterID::lowpass_freq.getParamID());
    
    ASSERT_NE(highpassFreq, nullptr);
    ASSERT_NE(midFreq, nullptr);
    ASSERT_NE(lowpassFreq, nullptr);
    
    // Parameters should be normalized between 0 and 1
    EXPECT_GE(highpassFreq->getValue(), 0.0f);
    EXPECT_LE(highpassFreq->getValue(), 1.0f);
    EXPECT_GE(midFreq->getValue(), 0.0f);
    EXPECT_LE(midFreq->getValue(), 1.0f);
    EXPECT_GE(lowpassFreq->getValue(), 0.0f);
    EXPECT_LE(lowpassFreq->getValue(), 1.0f);
}

// Test that plugin state can be saved and restored
TEST_F(AudioProcessorTest, StateManagement) {
    // Set some parameters to non-default values
    auto& params = processor->getParameters();
    auto highpassFreq = params.getParameter(audio_plugin::ParameterID::highpass_freq.getParamID());
    auto midFreq = params.getParameter(audio_plugin::ParameterID::mid_freq.getParamID());
    
    // Save the original values
    float originalHighpass = highpassFreq->getValue();
    float originalMid = midFreq->getValue();
    
    // Set to new values
    highpassFreq->setValueNotifyingHost(0.3f);
    midFreq->setValueNotifyingHost(0.7f);
    
    // Get state
    juce::MemoryBlock state;
    processor->getStateInformation(state);
    EXPECT_GT(state.getSize(), 0);
    
    // Create new processor and restore state
    auto newProcessor = std::make_unique<audio_plugin::AudioPluginAudioProcessor>();
    newProcessor->setStateInformation(state.getData(), static_cast<int>(state.getSize()));
    
    // Verify parameter values are restored
    auto& newParams = newProcessor->getParameters();
    auto newHighpassFreq = newParams.getParameter(audio_plugin::ParameterID::highpass_freq.getParamID());
    auto newMidFreq = newParams.getParameter(audio_plugin::ParameterID::mid_freq.getParamID());
    
    EXPECT_NEAR(newHighpassFreq->getValue(), 0.3f, 0.01f);
    EXPECT_NEAR(newMidFreq->getValue(), 0.7f, 0.01f);
    
    // Reset parameters to avoid affecting other tests
    highpassFreq->setValueNotifyingHost(originalHighpass);
    midFreq->setValueNotifyingHost(originalMid);
}

// Test the FIFO mechanism used by the plugin
TEST_F(AudioProcessorTest, FIFOTest) {
    // Test the FIFO mechanism used for FFT visualization
    
    // Create a FIFO for testing
    audio_plugin::Fifo<juce::AudioBuffer<float>> testFifo;
    testFifo.prepare(2, 512);
    
    // Create a test buffer
    juce::AudioBuffer<float> testBuffer = createTestBuffer(2, 512);
    
    // Push buffer into FIFO
    bool pushResult = testFifo.push(testBuffer);
    EXPECT_TRUE(pushResult);
    
    // Check read availability
    EXPECT_EQ(testFifo.getNumAvailableForReading(), 1);
    
    // Pull buffer from FIFO
    juce::AudioBuffer<float> outputBuffer;
    bool pullResult = testFifo.pull(outputBuffer);
    EXPECT_TRUE(pullResult);
    
    // Verify data integrity
    for (int channel = 0; channel < 2; ++channel) {
        for (int sample = 0; sample < 10; ++sample) {
            EXPECT_FLOAT_EQ(outputBuffer.getSample(channel, sample), testBuffer.getSample(channel, sample));
        }
    }
}

// Performance test - check if the plugin can process audio in real-time
TEST_F(AudioProcessorTest, PerformanceTest) {
    const int numIterations = 1000;
    juce::AudioBuffer<float> testBuffer = createTestBuffer(2, blockSize);
    juce::MidiBuffer midiBuffer;
    
    juce::Time startTime = juce::Time::getCurrentTime();
    
    // Process audio multiple times to simulate real-time processing
    for (int i = 0; i < numIterations; ++i) {
        processor->processBlock(testBuffer, midiBuffer);
}
    
    juce::RelativeTime processingTime = juce::Time::getCurrentTime() - startTime;
    double expectedMaxTime = (blockSize * numIterations) / sampleRate;
    
    // Processing should be faster than real-time
    EXPECT_LT(processingTime.inSeconds(), expectedMaxTime);
    
    // Output timing information
    std::cout << "Performance test: processed " << numIterations << " blocks in " 
              << processingTime.inSeconds() << " seconds (real-time would be " 
              << expectedMaxTime << " seconds)" << std::endl;
}

}  // namespace audio_plugin_test
