# HackTV "HackDAC"

This repository is a fork of the HackRF project which contains modified firmware and PCB files for the HackDAC.

![HackDAC Alpha](https://raw.github.com/inaxeon/hacktv-hackrf/master/hardware/hackdac-alpha/images/top.jpg)

# What is it?

HackDAC is an add-on board for the HackRF One intended to provide a broadcast quality baseband video and audio output for HackTV. HackDAC was developed to address a number of frustrations of HackTV users and developers. Firstly, the output of the HackRF is quite noisy, to the extent that it’s visible when viewing television signals generated by HackTV. Secondly it is sometimes more desirable to work with baseband video signals, particularly with devices which do not have an RF input.

# Why is it bristling with trimmable inductors? That looks rather dated!

Well yes, it is. For anybody wanting to design a device which has an analogue video output there are certainly more than a few types of chip for the job, however these (to the best of our current knowledge) are all "consumer video encoder" chips. While these devices are plentiful, inexpensive and produce good quality signals using just a single chip with a few passives at the output, unfortunately they do all of the sync generation and chrominance encoding internally, limiting the output typically to just PAL and NTSC (and SECAM in some rare cases).

HackTV on the other hand is a fully software defined TV transmitter which does not want the output hardware doing anything clever for it – this is why it is able to generate a massive range of TV signals, some quite obscure.

Ultimately HackTV wants a high-quality arbitrary video signal generator as its output and sadly there is no commercial reason for such a thing to exist, beyond analogue video test equipment, very little of which is designed or sold today.

In 2022 Danish Entrepreneur Karsten Hansen dumped 4 gigabytes of engineering materials from Philips TV & Test Equipment into a github repository (intellectual property he personally came to own). Contained within the repository was detailed information about how to build and calibrate a very high-quality arbitrary video signal generator circuit first used in the Philips PM5655. This circuit became the basis of "HackDAC Alpha" – the current design.

Unfortunately it's quite difficult to build, and calibrating it takes some practice, not to mention some rather specialised test equipment. Longer term the project will have to wave goodbye to this circuit however it is important for the time being, firstly because it works and proves the concept, secondly because its performance is exceptional.

# Specifications

## Video output
* Bandwidth: DC-6 MHz
* Voltage range: 2V pk-pk (+/- 1V)
* Output impedence: 75Ω
* Sample rate: 13.5 MHz (fixed, per CCIR 601)

## Sync output
* Output voltage: -1.8V (0v at rest)
* Rise time: 230ns
* Output impedence: 75Ω

## Audio output
* Output voltage: 2.1 V(RMS)
* Sample rate (with video): 210.9 KHz
* Sample rate (standalone): 48 KHz

# How do I obtain one?

The project is presently in the prototype stage.
