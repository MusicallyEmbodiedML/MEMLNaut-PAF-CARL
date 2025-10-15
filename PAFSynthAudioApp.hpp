#ifndef __PAF_SYNTH_AUDIO_APP_HPP__
#define __PAF_SYNTH_AUDIO_APP_HPP__

#include "src/memllib/audio/AudioAppBase.hpp" // Added missing include
#include "src/memllib/synth/maximilian.h" // Added missing include for maxiSettings, maxiOsc, maxiTrigger, maxiDelayline, maxiEnvGen, maxiLine

#include <cstddef>
#include <cstdint>
#include <memory> // Added for std::shared_ptr

#include "src/memllib/synth/maxiPAF.hpp"
#include "src/memllib/interface/InterfaceBase.hpp" // Added missing include


// #define ARPEGGIATOR
class ADSRLite {
    public:
    
    enum envStage{WAITTOTRIG, ATTACK, DECAY, SUSTAIN, RELEASE};

    void setup(float attackTimeMs, float decayTimeMs, float newSustainLevel, float releaseTimeMs) {
        attackIncFull = 1.f/((attackTimeMs / 1000.f) * kSampleRate);
        attackInc = attackIncFull;
        decayInc = 1.f/((decayTimeMs / 1000.f) * kSampleRate);
        sustainLevel = newSustainLevel;
        decayInc *= (1.f - sustainLevel);
        releaseMs = releaseTimeMs;
    }

    float play() {

        switch(stage) {
            case envStage::WAITTOTRIG:
            {
                envelopeValue = 0.f;
                break;
            }
            case envStage::ATTACK:
            {
                envelopeValue += attackInc;
                if (envelopeValue >= 1.f) {
                    stage = envStage::DECAY;
                    Serial.printf("decay %f\n", envelopeValue);
                }
                break;
            }
            case envStage::DECAY:
            {
                envelopeValue -= decayInc;
                if (envelopeValue<=sustainLevel) {
                    stage = envStage::SUSTAIN;
                    Serial.printf("sus %f\n", envelopeValue);
                }
                break;
            }
            case envStage::SUSTAIN:
            {
                // envelopeValue = sustainLevel;
                break;
            }
            case envStage::RELEASE:
            {
                envelopeValue -= relInc;
                if (envelopeValue <= 0.f) {
                    stage = envStage::WAITTOTRIG;
                    Serial.println("wait");
                    envelopeValue=0.f;
                }
                break;
            }
        }
        return envelopeValue * velocity;
    }

    void reset() {
        stage = envStage::WAITTOTRIG;
        envelopeValue=0;

    }

    void trigger(float vel) {
        stage = envStage::ATTACK;
        attackInc = attackIncFull * (1.f - envelopeValue);
        velocity = vel;
    }

    void release() {
        relInc = 1.f/((releaseMs / 1000.f) * kSampleRate);
        relInc *= envelopeValue;
        stage = envStage::RELEASE;
    }

private:
    envStage stage = envStage::WAITTOTRIG;
    float attackInc=0, attackIncFull=0, decayInc=0, sustainLevel=0,relInc=0, releaseMs;
    float envelopeValue=0;
    float velocity = 1.f;
};

template<size_t NPARAMS=33>
class PAFSynthAudioApp : public AudioAppBase<NPARAMS>
{
public:
    static constexpr size_t kN_Params = NPARAMS;
    static constexpr size_t nFREQs = 17;
    static constexpr float frequencies[nFREQs] = {100, 200, 400,800, 400, 800, 100,1600,100,400,100,50,1600,200,100,800,400};


    PAFSynthAudioApp() : AudioAppBase<NPARAMS>() {
    };

    bool __force_inline euclidean(float phase, const size_t n, const size_t k, const size_t offset, const float pulseWidth)
    {
        // Euclidean function
        const float fi = phase * n;
        int i = static_cast<int>(fi);
        const float rem = fi - i;
        if (i == n)
        {
            i--;
        }
        const int idx = ((i + n - offset) * k) % n;
        return (idx < k && rem < pulseWidth) ? 1 : 0;
    }

    stereosample_t __force_inline Process(const stereosample_t x) override
    {
        float x1[1];

        // const float trig = pulse.square(1);

        paf0.play(x1, 1, baseFreq, baseFreq + (paf0_cf * baseFreq), paf0_bw * baseFreq, paf0_vib, paf0_vfr, paf0_shift, 0);
        float p0 = *x1;

        const float freq1 = baseFreq * detune;

        paf1.play(x1, 1, freq1, freq1 + (paf1_cf * freq1), paf1_bw * freq1, paf1_vib, paf1_vfr, paf1_shift, 1);
        const float p1 = *x1;

        const float freq2 = freq1 * detune;

        paf2.play(x1, 1, freq2, freq2 + (paf2_cf * freq2), paf2_bw * freq2, paf2_vib, paf2_vfr, paf2_shift, 1);
        const float p2 = *x1;

        const float freq3 = freq2 * detune;

        paf3.play(x1, 1, freq3, freq3 + (paf3_cf * freq3), paf3_bw * freq3, paf3_vib, paf3_vfr, paf3_shift, 1);
        const float p3 = *x1;

        auto shapedSine = [](float phasor, float gain, float asym) -> float {
            // This function shapes the sine wave to create a foldback effect
            float x = sinf(phasor * TWOPI);
            x = sinf(((x * TWOPI) * gain) + asym);
            return x;
        };

        float y = p0; //+ p1 + p2 + p3;
    
        // float rm = p0 * p1 * p2 * p3;

        // y = y + (rm * rmGain);

        y = y + (shapedSine(y, sineShapeGain, sineShapeGain) * sineShapeMix);


    #ifdef ARPEGGIATOR
        const float ph = phasorOsc.phasor(1);
        const bool euclidNewNote = euclidean(ph, 12, euclidN, 0, 0.1f);
        if(zxdetect.onZX(euclidNewNote)) {
            envamp=0.8f;
            freqIndex++;
            if(freqIndex >= nFREQs) {
                freqIndex = 0;
            }
            baseFreq = frequencies[freqIndex];
    }else{
            // constexpr float envdec = 0.2f/9000.f;
            envamp -= envdec;
            if (envamp < 0.f) {
                envamp = 0.f;
            }
        }
    #else
        if(newNote) {
            newNote = false;
            envamp=0.8f;
    }else{
            // constexpr float envdec = 0.2f/9000.f;
            envamp -= envdec;
            if (envamp < 0.f) {
                envamp = 0.f;
            }
        }
    #endif
        float envval = env.play();
        y = y * envval;
        // y = y * envamp* envamp;

    #ifndef ARPEGGIATOR
        y *= noteVel;
    #endif

        float d1 = (dl1.play(y, 100, 0.8f) * dl1mix);
        // float d2 = (dl2.play(y, 15000, 0.8f) * dl2mix);
        y = y + d1;// + d2;
        stereosample_t ret { y, y };
        frame++;
        return ret;
    }

    void Setup(float sample_rate, std::shared_ptr<InterfaceBase> interface) override
    {
        AudioAppBase<NPARAMS>::Setup(sample_rate, interface);
        maxiSettings::sampleRate = sample_rate;


        paf0.init();
        paf0.setsr(maxiSettings::getSampleRate(), 1);
        // paf0.freq(100, 0);
        // // paf0.amp(1,0);
        // paf0.bw(200,0);
        // paf0.cf(210,0);
        // paf0.vfr(5,0);
        // paf0.vib(0.1,0);
        // paf0.shift(10,0);

        paf1.init();
        paf1.setsr(maxiSettings::getSampleRate(), 1);
        // paf1.freq(150, 0);
        // // paf1.amp(1,0);
        // paf1.bw(200,0);
        // paf1.cf(210,0);
        // paf1.vfr(5,0);
        // paf1.vib(0.1,0);
        // paf1.shift(10,0);

        paf2.init();
        paf2.setsr(maxiSettings::getSampleRate(), 1);

        paf3.init();
        paf3.setsr(maxiSettings::getSampleRate(), 1);

        // env.setupAR(10,100);
        arpFreq = frequencies[0];
        // line.prepare(1.f,0.f,100.f,false);
        // line.triggerEnable(true);
        envamp=1.f;

        env.setup(500,500,0.8,1000);

        queue_init(&qMIDINoteOn, sizeof(uint8_t)*2, 1);
        queue_init(&qMIDINoteOff, sizeof(uint8_t)*2, 1);

    }

    inline float mtof(uint8_t note) {
        // Convert MIDI note to frequency
        return 440.0f * exp2f((note - 69) / 12.0f);
    }

    void loop() override {
        uint8_t midimsg[2];
        if (firstParamsReceived && queue_try_remove(&qMIDINoteOn, &midimsg)) {
            // Serial.printf("PAFSynthAudioApp::ProcessParams - Received MIDI Note On: %d, Velocity: %d\n", midimsg[0], midimsg[1]);
            baseFreq = mtof(midimsg[0]);
            noteVel = midimsg[1] / 127.0f; // Normalize velocity to [0, 1]
            noteVel = noteVel * noteVel; // Square the velocity for more pronounced effect
            newNote = true;
            env.trigger(noteVel);
        }
        if (firstParamsReceived && queue_try_remove(&qMIDINoteOff, &midimsg)) {
            // Serial.printf("PAFSynthAudioApp::ProcessParams - Received MIDI Note On: %d, Velocity: %d\n", midimsg[0], midimsg[1]);
            env.release();
            Serial.println("release");
        }
        AudioAppBase<NPARAMS>::loop();
    }

    void ProcessParams(const std::array<float, NPARAMS>& params)
    {
        firstParamsReceived = true;
        // // Map parameters to the synth
        // synth_.mapParameters(params);
        // //Serial.print("Params processed.");
        // paf0_freq = 50.f + (params[0] * params[0] * 1000.f);
        // paf1_freq = 50.f + (params[1] * params[1] * 1000.f);

        // paf0_cf = arpFreq + (params[2] * params[2] * arpFreq * 1.f);
        // paf1_cf = arpFreq + (params[3] * params[3] * arpFreq * 1.f);
        // paf2_cf = arpFreq + (params[4] * params[4] * arpFreq * 1.f);
        paf0_cf = (params[2] * params[2]  * 0.1f);
        paf1_cf = (params[3] * params[3]  * 1.f);
        paf2_cf = (params[4] * params[4]  * 1.f);
        paf3_cf = (params[21] * params[21]  * 1.f);

        // paf0_bw = 5.f + (params[5] * arpFreq * 0.5f);
        // paf1_bw = 5.f + (params[6] * arpFreq * 0.5f);
        // paf2_bw = 5.f + (params[7] * arpFreq * 0.5f);
        paf0_bw = 0.1f + (params[5] * 0.2f);
        paf1_bw = 0.1f + (params[6] * 2.f);
        paf2_bw = 0.1f + (params[7] * 2.f);
        paf3_bw = 0.1f + (params[22] * 2.f);

        paf0_vib = (params[8] * params[8] * 0.1f);
        paf1_vib = (params[9] * params[9] * 0.99f);
        paf2_vib = (params[10] * params[10] * 0.99f);
        paf3_vib = (params[23] * params[23] * 0.99f);

        paf0_vfr = (params[11] * params[11]* 0.1f);
        paf1_vfr = (params[12] * params[12] * 10.f);
        paf2_vfr = (params[13] * params[13] * 10.f);
        paf3_vfr = (params[24] * params[24] * 10.f);

        paf0_shift = (params[14] * 10.f);
        paf1_shift = (params[15] * 1000.f);
        paf2_shift = (params[16] * 1000.f);
        paf3_shift = (params[25] * 1000.f);

        dl1mix = params[17] * params[17] * 0.4f;
        // dl2mix = params[18] * params[18] * 0.4f;
        detune = 1.0f + (params[18] * 0.1);

        euclidN = static_cast<size_t>(2 + (params[19] * 5));

        // envdec=((params[20] * 1.f) + 0.01f)/9000.f; // Decay rate for the envelope
        env.setup(params[30] * 200.f,params[20] * params[20] * 500.f, params[31] * 0.5f, params[32] * 500.f );

        sineShapeGain = params[26] * params[26];
        sineShapeASym = params[27] * params[27] * 0.1f;
        sineShapeMix = params[28];

        rmGain = params[29] * params[29];
        // sineShapeMixInv = 1.f-sineShapeMix;
        // Serial.printf("%f %f %f %f %f\n", paf0_cf,  paf0_bw, paf0_vib, paf0_vfr, paf0_shift);
    }

    queue_t qMIDINoteOn, qMIDINoteOff;

protected:

    maxiPAFOperator paf0;
    maxiPAFOperator paf1;
    maxiPAFOperator paf2;
    maxiPAFOperator paf3;

    maxiDelayline<5000> dl1;
    maxiDelayline<15100> dl2;

    maxiOsc pulse;
    // maxiEnvGen env;

    ADSRLite env;

    float frame=0;

    float paf0_freq = 100;
    float paf1_freq = 100;
    float paf2_freq = 50;
    float paf3_freq = 50;

    float paf0_cf = 200;
    float paf1_cf = 250;
    float paf2_cf = 250;
    float paf3_cf = 250;

    float paf0_bw = 100;
    float paf1_bw = 5000;
    float paf2_bw = 5000;
    float paf3_bw = 5000;

    float paf0_vib = 0;
    float paf1_vib = 1;
    float paf2_vib = 1;
    float paf3_vib = 1;

    float paf0_vfr = 2;
    float paf1_vfr = 2;
    float paf2_vfr = 2;
    float paf3_vfr = 2;

    float paf0_shift = 0;
    float paf1_shift = 0;
    float paf2_shift = 0;
    float paf3_shift = 0;

    float dl1mix = 0.0f;
    float dl2mix = 0.0f;

    float rmGain = 0.f;
    
    float sineShapeGain=0.1;
    float sineShapeASym = 0.f;
    float sineShapeMix = 0.f;
    float sineShapeMixInv = 1.f;
    size_t counter=0;
    size_t freqIndex = 0;
    size_t freqOffset = 0;
    float arpFreq=50;

    maxiLine line;
    float envamp=0.f;

    float detune = 1.0;

    maxiOsc phasorOsc;
    maxiTrigger zxdetect;

    size_t euclidN=4;

    float baseFreq = 50.0f; // Base frequency for the synth
    bool newNote=false;
    float noteVel = 0.f;
    bool firstParamsReceived = false;

    float envdec=0.2f/9000.f; // Decay rate for the envelope

};

#endif  // __PAF_SYNTH_AUDIO_APP_HPP__
