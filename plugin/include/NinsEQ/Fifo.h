#pragma once

#include <array>
#include <vector>
#include <juce_audio_processors/juce_audio_processors.h>

namespace audio_plugin {

template<typename T>
class Fifo
{
public:
    void prepare(int numElements)
    {
        for (auto& buffer : buffers)
        {
            buffer.clear();
            buffer.resize(numElements, T());
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
    static constexpr int capacity = 30;
    std::array<T, capacity> buffers;
    juce::AbstractFifo fifo{capacity};
};

template<typename BlockType>
class SingleChannelSampleFifo
{
public:
    SingleChannelSampleFifo(int numChannels) : numChannels(numChannels) 
    {
        prepared.store(false);
    }
    
    void update(const BlockType& buffer)
    {
        jassert(prepared.load());
        jassert(buffer.getNumChannels() > numChannels);
        auto* channelPtr = buffer.getReadPointer(numChannels);
        
        for (int i = 0; i < buffer.getNumSamples(); ++i)
        {
            pushNextSampleIntoFifo(channelPtr[i]);
        }
    }

    void prepare(int bufferSize)
    {
        prepared.store(false);
        size.store(bufferSize);
        
        bufferToFill.setSize(1, bufferSize, false, true, true);
        audioBufferFifo.prepare(bufferSize);
        fifoIndex = 0;
        prepared.store(true);
    }
    
    //==============================================================================
    int getNumCompleteBuffersAvailable() const { return audioBufferFifo.getNumAvailableForReading(); }
    bool isPrepared() const { return prepared.load(); }
    int getSize() const { return size.load(); }
    //==============================================================================
    bool getAudioBuffer(BlockType& buf) { return audioBufferFifo.pull(buf); }
private:
    int numChannels;
    int fifoIndex = 0;
    Fifo<BlockType> audioBufferFifo;
    BlockType bufferToFill;
    std::atomic<bool> prepared = false;
    std::atomic<int> size = 0;
    
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

} // namespace audio_plugin 