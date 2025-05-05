#include "NinsEQ/PluginEditor.h"
#include "NinsEQ/PluginProcessor.h"

namespace audio_plugin {

// PathProducer Implementation
void PathProducer::process(juce::Rectangle<float> fftBounds, double sampleRate)
{
    juce::AudioBuffer<float> tempIncomingBuffer;
    
    while (channelFifo->getNumCompleteBuffersAvailable() > 0)
    {
        if (channelFifo->getAudioBuffer(tempIncomingBuffer))
        {
            auto size = tempIncomingBuffer.getNumSamples();
            
            juce::FloatVectorOperations::copy(
                monoBuffer.getWritePointer(0, 0),
                monoBuffer.getReadPointer(0, size),
                monoBuffer.getNumSamples() - size);
            
            juce::FloatVectorOperations::copy(
                monoBuffer.getWritePointer(0, monoBuffer.getNumSamples() - size),
                tempIncomingBuffer.getReadPointer(0, 0),
                size);
            
            fftDataGenerator.produceFFTDataForRendering(monoBuffer, -48.f);
        }
    }
    
    const auto fftSize = fftDataGenerator.getFFTSize();
    const auto binWidth = sampleRate / (double)fftSize;
    
    while (fftDataGenerator.getNumAvailableFFTDataBlocks() > 0)
    {
        std::vector<float> fftData;
        if (fftDataGenerator.getFFTData(fftData))
        {
            pathProducer.generatePath(fftData, fftBounds, fftSize, binWidth, -48.f);
        }
    }
    
    while (pathProducer.getNumPathsAvailable())
    {
        pathProducer.getPath(channelFFTPath);
    }
}

// FrequencyResponseCurveComponent Implementation
FrequencyResponseCurveComponent::FrequencyResponseCurveComponent(AudioPluginAudioProcessor& p)
    : audioProcessor(p),
      leftPathProducer(audioProcessor.leftChannelFifo),
      rightPathProducer(audioProcessor.rightChannelFifo)
{
    startTimerHz(30); // Update at 30fps
}

FrequencyResponseCurveComponent::~FrequencyResponseCurveComponent()
{
    stopTimer();
}

void FrequencyResponseCurveComponent::paint(juce::Graphics& g)
{
    // Background - change to dark blue
    g.fillAll(juce::Colour(20, 25, 40));
    
    auto bounds = getLocalBounds();
    
    // Draw frequency grid lines
    g.setColour(juce::Colour(40, 50, 80));
    
    // Frequency markers at standard values
    const int frequencyMarks[] = {20, 30, 40, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
    const int numFrequencyMarks = sizeof(frequencyMarks) / sizeof(frequencyMarks[0]);
    
    for (int i = 0; i < numFrequencyMarks; ++i)
    {
        const double freq = frequencyMarks[i];
        const double normX = mapFrequencyToX(freq, bounds.getWidth());
        const float x = bounds.getX() + normX * bounds.getWidth();
        
        // Only draw lines for major frequency divisions
        if (freq == 20 || freq == 50 || freq == 100 || freq == 200 || freq == 500 || 
            freq == 1000 || freq == 2000 || freq == 5000 || freq == 10000 || freq == 20000)
        {
            g.drawVerticalLine(std::round(x), bounds.getY(), bounds.getBottom());
        }
        
        // Draw labels for major frequencies
        if (freq == 20 || freq == 100 || freq == 1000 || freq == 10000 || freq == 20000)
        {
            g.setColour(juce::Colour(150, 180, 220));
            g.setFont(13.0f);
            
            juce::String freqText;
            if (freq >= 1000)
                freqText = juce::String(freq / 1000.0, 1) + "k";
            else
                freqText = juce::String(freq);
                
            g.drawText(freqText, x - 15, bounds.getBottom() - 20, 30, 20, 
                       juce::Justification::centred, false);
                       
            g.setColour(juce::Colour(40, 50, 80));
        }
    }
    
    // Draw gain markers (dB)
    const int gainMarks[] = {-24, -18, -12, -6, 0, 6, 12, 18, 24};
    const int numGainMarks = sizeof(gainMarks) / sizeof(gainMarks[0]);
    
    for (int i = 0; i < numGainMarks; ++i)
    {
        const double gain = gainMarks[i];
        const double normY = mapDbToY(gain, bounds.getHeight());
        const float y = bounds.getY() + (1.0f - normY) * bounds.getHeight();
        
        g.drawHorizontalLine(std::round(y), bounds.getX(), bounds.getRight());
        
        if (gain == 0 || (int)gain % 12 == 0) // Only label at 0, ±12, ±24 dB
        {
            g.setColour(juce::Colour(150, 180, 220));
            g.setFont(13.0f);
            g.drawText(juce::String(gain) + " dB", bounds.getX() + 3, y - 10, 50, 20, 
                       juce::Justification::left, false);
            g.setColour(juce::Colour(40, 50, 80));
        }
    }
    
    // Draw the filter response curve first (as background)
    drawFilterResponse(g, bounds);
    
    // Process FFT data
    auto fftBounds = bounds.toFloat();
    auto sampleRate = audioProcessor.getSampleRate();
    
    leftPathProducer.process(fftBounds, sampleRate);
    rightPathProducer.process(fftBounds, sampleRate);
    
    // Draw left channel FFT path
    g.setColour(juce::Colour(70, 130, 180).withAlpha(0.5f)); // Blue for left channel
    g.strokePath(leftPathProducer.getPath(), juce::PathStrokeType(1.f));
    
    // Draw right channel FFT path
    g.setColour(juce::Colour(100, 200, 250).withAlpha(0.5f)); // Lighter blue for right channel
    g.strokePath(rightPathProducer.getPath(), juce::PathStrokeType(1.f));
    
    // Draw gain reduction meter with improved visibility
    float gainReduction = audioProcessor.getCurrentGainReduction();
    
    if (gainReduction < 1.0f) // Only show when compressing
    {
        int meterWidth = 12;
        int meterHeight = bounds.getHeight() / 3;
        int meterX = bounds.getRight() - meterWidth - 5;
        int meterY = bounds.getCentreY() - meterHeight / 2;
        
        g.setColour(juce::Colours::black.withAlpha(0.7f));
        g.fillRect(meterX, meterY, meterWidth, meterHeight);
        
        float reductionDb = juce::Decibels::gainToDecibels(gainReduction);
        float reductionNormalized = juce::jmap(reductionDb, -20.0f, 0.0f, 0.0f, 1.0f);
        
        g.setGradientFill(juce::ColourGradient(
            juce::Colour(220, 50, 50),  // Red
            meterX, meterY,
            juce::Colour(50, 180, 255), // Blue
            meterX, meterY + meterHeight,
            false));
            
        int reductionHeight = std::round(meterHeight * reductionNormalized);
        g.fillRect(meterX, meterY + meterHeight - reductionHeight, meterWidth, reductionHeight);
        
        // Draw a label for the meter
        g.setColour(juce::Colours::white);
        g.setFont(12.0f);
        g.drawText("GR", meterX - 18, meterY + meterHeight / 2 - 10, 20, 20, juce::Justification::centred);
    }
}

void FrequencyResponseCurveComponent::drawFilterResponse(juce::Graphics& g, const juce::Rectangle<int>& bounds)
{
    using namespace juce;
    
    // Get current parameters
    const double sampleRate = audioProcessor.getSampleRate();
    
    // Don't draw if sampling rate isn't available yet
    if (sampleRate <= 0)
        return;
    
    // Define frequency range for the curve
    const double minFreq = 20.0;
    const double maxFreq = 20000.0;
    
    // Number of points to evaluate the curve at
    const int numPoints = bounds.getWidth();
    
    // Get current parameters
    const float highpassFreq = audioProcessor.getParameters().getParameter(ParameterID::highpass_freq.getParamID())->getValue();
    const float highpassQ = audioProcessor.getParameters().getParameter(ParameterID::highpass_q.getParamID())->getValue();
    const float midFreq = audioProcessor.getParameters().getParameter(ParameterID::mid_freq.getParamID())->getValue();
    const float midQ = audioProcessor.getParameters().getParameter(ParameterID::mid_q.getParamID())->getValue();
    const float midGain = audioProcessor.getParameters().getParameter(ParameterID::mid_gain.getParamID())->getValue();
    const float lowpassFreq = audioProcessor.getParameters().getParameter(ParameterID::lowpass_freq.getParamID())->getValue();
    const float lowpassQ = audioProcessor.getParameters().getParameter(ParameterID::lowpass_q.getParamID())->getValue();
    
    // Convert from normalized values
    const float highpassFreqHz = audioProcessor.getParameters().getParameterRange(ParameterID::highpass_freq.getParamID()).convertFrom0to1(highpassFreq);
    const float highpassQValue = audioProcessor.getParameters().getParameterRange(ParameterID::highpass_q.getParamID()).convertFrom0to1(highpassQ);
    const float midFreqHz = audioProcessor.getParameters().getParameterRange(ParameterID::mid_freq.getParamID()).convertFrom0to1(midFreq);
    const float midQValue = audioProcessor.getParameters().getParameterRange(ParameterID::mid_q.getParamID()).convertFrom0to1(midQ);
    const float midGainDb = audioProcessor.getParameters().getParameterRange(ParameterID::mid_gain.getParamID()).convertFrom0to1(midGain);
    const float lowpassFreqHz = audioProcessor.getParameters().getParameterRange(ParameterID::lowpass_freq.getParamID()).convertFrom0to1(lowpassFreq);
    const float lowpassQValue = audioProcessor.getParameters().getParameterRange(ParameterID::lowpass_q.getParamID()).convertFrom0to1(lowpassQ);
    
    // Get current gain reduction for visualization
    float gainReduction = audioProcessor.getCurrentGainReduction();
    float dynamicGainDb = midGainDb * gainReduction; // Apply gain reduction
    
    // Draw original (static) filter response
    {
        // Create the filter coefficients to compute the response
        auto highpassCoeffs = dsp::IIR::Coefficients<float>::makeHighPass(sampleRate, highpassFreqHz, highpassQValue);
        auto bandpassCoeffs = dsp::IIR::Coefficients<float>::makePeakFilter(sampleRate, midFreqHz, midQValue, Decibels::decibelsToGain(midGainDb));
        auto lowpassCoeffs = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, lowpassFreqHz, lowpassQValue);
        
        // For each point in the response curve
        Path originalResponseCurve;
        
        for (int i = 0; i < numPoints; ++i)
        {
            // Calculate the frequency at this point using logarithmic scale
            double normX = (double)i / (double)(numPoints - 1);
            double freq = minFreq * std::pow(maxFreq / minFreq, normX);
            
            // Compute the magnitude response of each filter at this frequency
            double highpassMag = highpassCoeffs->getMagnitudeForFrequency(freq, sampleRate);
            double bandpassMag = bandpassCoeffs->getMagnitudeForFrequency(freq, sampleRate);
            double lowpassMag = lowpassCoeffs->getMagnitudeForFrequency(freq, sampleRate);
            
            // Calculate the combined magnitude by multiplying the individual magnitudes
            double magnitude = highpassMag * bandpassMag * lowpassMag;
            
            // Convert to decibels and map to y position
            double magnitudeDb = Decibels::gainToDecibels(magnitude);
            double normY = mapDbToY(magnitudeDb, bounds.getHeight());
            
            // Calculate the x and y coordinates
            float x = bounds.getX() + normX * bounds.getWidth();
            float y = bounds.getY() + (1.0f - normY) * bounds.getHeight();
            
            // Add the point to the path
            if (i == 0)
                originalResponseCurve.startNewSubPath(x, y);
            else
                originalResponseCurve.lineTo(x, y);
        }
        
        // Draw the original response curve with semi-transparency
        if (gainReduction < 1.0f) {
            g.setColour(juce::Colour(130, 160, 255).withAlpha(0.4f)); // Light blue for original curve
            g.strokePath(originalResponseCurve, juce::PathStrokeType(1.5f));
        }
    }
    
    // Create the filter coefficients for dynamic curve
    auto highpassCoeffs = dsp::IIR::Coefficients<float>::makeHighPass(sampleRate, highpassFreqHz, highpassQValue);
    auto bandpassCoeffs = dsp::IIR::Coefficients<float>::makePeakFilter(sampleRate, midFreqHz, midQValue, Decibels::decibelsToGain(dynamicGainDb));
    auto lowpassCoeffs = dsp::IIR::Coefficients<float>::makeLowPass(sampleRate, lowpassFreqHz, lowpassQValue);
    
    // For each point in the response curve
    Path responseCurve;
    
    for (int i = 0; i < numPoints; ++i)
    {
        // Calculate the frequency at this point using logarithmic scale
        double normX = (double)i / (double)(numPoints - 1);
        double freq = minFreq * std::pow(maxFreq / minFreq, normX);
        
        // Compute the magnitude response of each filter at this frequency
        double highpassMag = highpassCoeffs->getMagnitudeForFrequency(freq, sampleRate);
        double bandpassMag = bandpassCoeffs->getMagnitudeForFrequency(freq, sampleRate);
        double lowpassMag = lowpassCoeffs->getMagnitudeForFrequency(freq, sampleRate);
        
        // Calculate the combined magnitude by multiplying the individual magnitudes
        double magnitude = highpassMag * bandpassMag * lowpassMag;
        
        // Convert to decibels and map to y position
        double magnitudeDb = Decibels::gainToDecibels(magnitude);
        double normY = mapDbToY(magnitudeDb, bounds.getHeight());
        
        // Calculate the x and y coordinates
        float x = bounds.getX() + normX * bounds.getWidth();
        float y = bounds.getY() + (1.0f - normY) * bounds.getHeight();
        
        // Add the point to the path
        if (i == 0)
            responseCurve.startNewSubPath(x, y);
        else
            responseCurve.lineTo(x, y);
    }
    
    // Draw a fatter line with drop shadow for better visibility
    g.setColour(juce::Colours::black.withAlpha(0.5f));
    g.strokePath(responseCurve, juce::PathStrokeType(3.0f));
    
    g.setColour(juce::Colour(120, 210, 255)); // Bright blue for main curve
    g.strokePath(responseCurve, juce::PathStrokeType(2.0f));
    
    // Draw filter points/handles
    int handleSize = 10;
    
    // Highpass point
    float highpassX = bounds.getX() + mapFrequencyToX(highpassFreqHz, bounds.getWidth()) * bounds.getWidth();
    float highpassY = bounds.getCentreY(); // At 0dB
    g.setColour(juce::Colour(80, 120, 200)); // Blue
    g.fillEllipse(highpassX - handleSize/2, highpassY - handleSize/2, handleSize, handleSize);
    g.setColour(juce::Colours::white);
    g.drawEllipse(highpassX - handleSize/2, highpassY - handleSize/2, handleSize, handleSize, 1.0f);
    
    // Mid band point
    float midX = bounds.getX() + mapFrequencyToX(midFreqHz, bounds.getWidth()) * bounds.getWidth();
    float midY = bounds.getY() + (1.0f - mapDbToY(dynamicGainDb, bounds.getHeight())) * bounds.getHeight();
    g.setColour(juce::Colour(50, 180, 255)); // Brighter blue
    g.fillEllipse(midX - handleSize/2, midY - handleSize/2, handleSize, handleSize);
    g.setColour(juce::Colours::white);
    g.drawEllipse(midX - handleSize/2, midY - handleSize/2, handleSize, handleSize, 1.0f);
    
    // Draw a shadow handle to show the non-compressed state
    if (gainReduction < 1.0f) {
        float originalMidY = bounds.getY() + (1.0f - mapDbToY(midGainDb, bounds.getHeight())) * bounds.getHeight();
        g.setColour(juce::Colour(180, 220, 255).withAlpha(0.7f)); // Light blue
        g.drawEllipse(midX - handleSize/2, originalMidY - handleSize/2, handleSize, handleSize, 1.0f);
        
        // Draw a line connecting the two states - make it more visible
        g.setColour(juce::Colour(100, 180, 255).withAlpha(0.6f));
        g.drawLine(midX, originalMidY, midX, midY, 1.5f);
        
        // Add "gain reduction" indicator
        float reductionDb = midGainDb - dynamicGainDb;
        if (reductionDb > 0.5f) {
            g.setColour(juce::Colours::white);
            g.setFont(juce::Font(12.0f, juce::Font::bold));
            g.drawText(juce::String(reductionDb, 1) + " dB", 
                      midX + 10, (originalMidY + midY) / 2 - 10, 
                      50, 20, juce::Justification::left);
        }
    }
    
    // Lowpass point
    float lowpassX = bounds.getX() + mapFrequencyToX(lowpassFreqHz, bounds.getWidth()) * bounds.getWidth();
    float lowpassY = bounds.getCentreY(); // At 0dB
    g.setColour(juce::Colour(30, 100, 180)); // Darker blue
    g.fillEllipse(lowpassX - handleSize/2, lowpassY - handleSize/2, handleSize, handleSize);
    g.setColour(juce::Colours::white);
    g.drawEllipse(lowpassX - handleSize/2, lowpassY - handleSize/2, handleSize, handleSize, 1.0f);
}

void FrequencyResponseCurveComponent::resized()
{
    // Nothing specific needed here
}

void FrequencyResponseCurveComponent::updateCurve()
{
    // The curve is now drawn directly in the paint method
    repaint();
}

void FrequencyResponseCurveComponent::timerCallback()
{
    // Only trigger a repaint when audio is playing
    auto sampleRate = audioProcessor.getSampleRate();
    if (sampleRate > 0) {
        auto fftBounds = getLocalBounds().toFloat();
        leftPathProducer.process(fftBounds, sampleRate);
        rightPathProducer.process(fftBounds, sampleRate);
    }
    
    // Trigger a repaint to refresh everything
    repaint();
}

// Helper methods for coordinate mapping
double FrequencyResponseCurveComponent::mapFrequencyToX(double freq, int width) const
{
    // Map frequency to x coordinate using logarithmic scale
    return std::log10(freq / 20.0) / std::log10(20000.0 / 20.0);
}

double FrequencyResponseCurveComponent::mapDbToY(double db, int height) const
{
    // Map decibels to normalized y coordinate
    return juce::jmap(db, -24.0, 24.0, 0.0, 1.0);
}

// AudioPluginAudioProcessorEditor Implementation
AudioPluginAudioProcessorEditor::AudioPluginAudioProcessorEditor(
    AudioPluginAudioProcessor& p)
    : AudioProcessorEditor(&p), processorRef(p), responseCurveComponent(p)
{
    // Set a modern blue look and feel
    getLookAndFeel().setColour(juce::Slider::thumbColourId, juce::Colour(80, 170, 255));
    getLookAndFeel().setColour(juce::Slider::rotarySliderFillColourId, juce::Colour(50, 90, 150));
    getLookAndFeel().setColour(juce::Slider::rotarySliderOutlineColourId, juce::Colour(30, 50, 90));
    getLookAndFeel().setColour(juce::Label::textColourId, juce::Colour(180, 210, 255));

    // Set up slider styling
    auto setupSlider = [this](juce::Slider& slider, juce::Label& label, const juce::String& labelText) {
        slider.setSliderStyle(juce::Slider::RotaryVerticalDrag);
        slider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
        slider.setPopupDisplayEnabled(true, true, this);
        slider.setColour(juce::Slider::textBoxTextColourId, juce::Colour(180, 210, 255));
        slider.setColour(juce::Slider::textBoxOutlineColourId, juce::Colour(40, 60, 100));
        slider.setColour(juce::Slider::textBoxBackgroundColourId, juce::Colour(20, 30, 50));
        addAndMakeVisible(slider);
        
        label.setText(labelText, juce::dontSendNotification);
        label.setJustificationType(juce::Justification::centred);
        label.setColour(juce::Label::textColourId, juce::Colour(150, 200, 255));
        label.setFont(juce::Font(14.0f, juce::Font::bold));
        addAndMakeVisible(label);
    };
    
    // Initialize sliders
    setupSlider(highpassFreqSlider, highpassFreqLabel, "HP Freq");
    setupSlider(highpassQSlider, highpassQLabel, "HP Q");
    setupSlider(midFreqSlider, midFreqLabel, "Mid Freq");
    setupSlider(midQSlider, midQLabel, "Mid Q");
    setupSlider(midGainSlider, midGainLabel, "Mid Gain");
    setupSlider(midThresholdSlider, midThresholdLabel, "Threshold");
    setupSlider(midRatioSlider, midRatioLabel, "Ratio");
    setupSlider(midAttackSlider, midAttackLabel, "Attack");
    setupSlider(midReleaseSlider, midReleaseLabel, "Release");
    setupSlider(lowpassFreqSlider, lowpassFreqLabel, "LP Freq");
    setupSlider(lowpassQSlider, lowpassQLabel, "LP Q");
    
    // Make parameter attachments
    auto& parameters = processorRef.getParameters();
    highpassFreqAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::highpass_freq.getParamID(), highpassFreqSlider);
    highpassQAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::highpass_q.getParamID(), highpassQSlider);
    midFreqAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_freq.getParamID(), midFreqSlider);
    midQAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_q.getParamID(), midQSlider);
    midGainAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_gain.getParamID(), midGainSlider);
    midThresholdAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_threshold.getParamID(), midThresholdSlider);
    midRatioAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_ratio.getParamID(), midRatioSlider);
    midAttackAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_attack.getParamID(), midAttackSlider);
    midReleaseAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::mid_release.getParamID(), midReleaseSlider);
    lowpassFreqAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::lowpass_freq.getParamID(), lowpassFreqSlider);
    lowpassQAttachment = std::make_unique<SliderAttachment>(parameters, ParameterID::lowpass_q.getParamID(), lowpassQSlider);
    
    // Add frequency response component
    addAndMakeVisible(responseCurveComponent);
    
    // Set editor size
    setSize(800, 600);
    
    // Start timer for FFT updates
    startTimerHz(30);
}

AudioPluginAudioProcessorEditor::~AudioPluginAudioProcessorEditor() 
{
    stopTimer();
}

void AudioPluginAudioProcessorEditor::paint(juce::Graphics& g) {
    // Draw background
    g.fillAll(juce::Colour(15, 20, 35)); // Dark blue background
    
    // Draw title
    g.setColour(juce::Colour(150, 200, 255)); // Light blue text
    g.setFont(juce::Font(20.0f, juce::Font::bold));
    g.drawText("NinsEQ Dynamic Equalizer", 20, 10, getWidth() - 40, 30, juce::Justification::centred);
}

void AudioPluginAudioProcessorEditor::resized() {
    auto bounds = getLocalBounds().reduced(10);
    
    // Set aside top area for the spectrum display
    auto spectrumArea = bounds.removeFromTop(300);
    responseCurveComponent.setBounds(spectrumArea);
    
    // Add spacing between spectrum and controls
    bounds.removeFromTop(20);
    
    // Divide the remaining area for controls
    auto controlsArea = bounds;
    
    // Create sections for each filter group
    auto highpassArea = controlsArea.removeFromLeft(controlsArea.getWidth() * 0.25f);
    auto midArea = controlsArea.removeFromLeft(controlsArea.getWidth() * 0.6f);
    auto lowpassArea = controlsArea;
    
    // Calculate label height and spacing
    const int labelHeight = 20;
    const int labelSpacing = 5;
    
    // Layout the highpass controls
    juce::Rectangle<int> highpassLabelBounds(highpassArea.getX(), highpassArea.getY(), 
                                             highpassArea.getWidth(), labelHeight);
    juce::Rectangle<int> highpassSliderArea = highpassArea.withTrimmedTop(labelHeight + labelSpacing);
    
    // Layout highpass frequency control
    auto highpassFreqArea = highpassSliderArea.removeFromLeft(highpassSliderArea.getWidth() / 2).reduced(5);
    highpassFreqLabel.setBounds(highpassFreqArea.getX(), highpassFreqArea.getY() - labelHeight, 
                               highpassFreqArea.getWidth(), labelHeight);
    highpassFreqSlider.setBounds(highpassFreqArea);
    
    // Layout highpass Q control
    auto highpassQArea = highpassSliderArea.reduced(5);
    highpassQLabel.setBounds(highpassQArea.getX(), highpassQArea.getY() - labelHeight, 
                            highpassQArea.getWidth(), labelHeight);
    highpassQSlider.setBounds(highpassQArea);
    
    // Layout the mid band controls
    auto midTopRow = midArea.removeFromTop(midArea.getHeight() / 2);
    
    // Layout mid frequency, Q, and gain controls
    int midTopSliderWidth = midTopRow.getWidth() / 3;
    
    // Mid frequency
    auto midFreqArea = midTopRow.removeFromLeft(midTopSliderWidth).reduced(5);
    midFreqLabel.setBounds(midFreqArea.getX(), midFreqArea.getY() - labelHeight, 
                          midFreqArea.getWidth(), labelHeight);
    midFreqSlider.setBounds(midFreqArea);
    
    // Mid Q
    auto midQArea = midTopRow.removeFromLeft(midTopSliderWidth).reduced(5);
    midQLabel.setBounds(midQArea.getX(), midQArea.getY() - labelHeight, 
                       midQArea.getWidth(), labelHeight);
    midQSlider.setBounds(midQArea);
    
    // Mid gain
    auto midGainArea = midTopRow.reduced(5);
    midGainLabel.setBounds(midGainArea.getX(), midGainArea.getY() - labelHeight, 
                          midGainArea.getWidth(), labelHeight);
    midGainSlider.setBounds(midGainArea);
    
    // Layout mid dynamics controls
    int dynamicsSliderWidth = midArea.getWidth() / 4;
    
    // Threshold
    auto thresholdArea = midArea.removeFromLeft(dynamicsSliderWidth).reduced(5);
    midThresholdLabel.setBounds(thresholdArea.getX(), thresholdArea.getY() - labelHeight, 
                               thresholdArea.getWidth(), labelHeight);
    midThresholdSlider.setBounds(thresholdArea);
    
    // Ratio
    auto ratioArea = midArea.removeFromLeft(dynamicsSliderWidth).reduced(5);
    midRatioLabel.setBounds(ratioArea.getX(), ratioArea.getY() - labelHeight, 
                           ratioArea.getWidth(), labelHeight);
    midRatioSlider.setBounds(ratioArea);
    
    // Attack
    auto attackArea = midArea.removeFromLeft(dynamicsSliderWidth).reduced(5);
    midAttackLabel.setBounds(attackArea.getX(), attackArea.getY() - labelHeight, 
                            attackArea.getWidth(), labelHeight);
    midAttackSlider.setBounds(attackArea);
    
    // Release
    auto releaseArea = midArea.reduced(5);
    midReleaseLabel.setBounds(releaseArea.getX(), releaseArea.getY() - labelHeight, 
                             releaseArea.getWidth(), labelHeight);
    midReleaseSlider.setBounds(releaseArea);
    
    // Layout the lowpass controls
    juce::Rectangle<int> lowpassLabelBounds(lowpassArea.getX(), lowpassArea.getY(), 
                                           lowpassArea.getWidth(), labelHeight);
    juce::Rectangle<int> lowpassSliderArea = lowpassArea.withTrimmedTop(labelHeight + labelSpacing);
    
    // Layout lowpass frequency control
    auto lowpassFreqArea = lowpassSliderArea.removeFromLeft(lowpassSliderArea.getWidth() / 2).reduced(5);
    lowpassFreqLabel.setBounds(lowpassFreqArea.getX(), lowpassFreqArea.getY() - labelHeight, 
                              lowpassFreqArea.getWidth(), labelHeight);
    lowpassFreqSlider.setBounds(lowpassFreqArea);
    
    // Layout lowpass Q control
    auto lowpassQArea = lowpassSliderArea.reduced(5);
    lowpassQLabel.setBounds(lowpassQArea.getX(), lowpassQArea.getY() - labelHeight, 
                           lowpassQArea.getWidth(), labelHeight);
    lowpassQSlider.setBounds(lowpassQArea);
}

void AudioPluginAudioProcessorEditor::timerCallback()
{
    // Trigger repaint to update the spectrum analyzer
    responseCurveComponent.repaint();
}

}  // namespace audio_plugin
