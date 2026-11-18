"""Turns a folder of 16 kHz wavs into augmented microWakeWord features (training/validation/testing ragged mmaps).
Settings follow microWakeWord's basic_training_notebook."""
import argparse
import os

from mmap_ninja.ragged import RaggedMmap
from microwakeword.audio.augmentation import Augmentation
from microwakeword.audio.clips import Clips
from microwakeword.audio.spectrograms import SpectrogramGeneration

parser = argparse.ArgumentParser()
parser.add_argument("wav_dir")
parser.add_argument("out_dir")
parser.add_argument("--augment_dir", default="/root/mww/augment")
args = parser.parse_args()

clips = Clips(input_directory=args.wav_dir, file_pattern="**/*.wav", max_clip_duration_s=None,
              remove_silence=False, random_split_seed=10, split_count=0.1)
augmenter = Augmentation(
    augmentation_duration_s=3.2,
    augmentation_probabilities={"SevenBandParametricEQ": 0.1, "TanhDistortion": 0.1, "PitchShift": 0.1,
                                "BandStopFilter": 0.1, "AddColorNoise": 0.1, "AddBackgroundNoise": 0.75,
                                "Gain": 1.0, "RIR": 0.5},
    impulse_paths=[os.path.join(args.augment_dir, "mit_rirs")],
    background_paths=[os.path.join(args.augment_dir, d) for d in ("fma_16k", "audioset_16k")],
    background_min_snr_db=-5, background_max_snr_db=10, min_jitter_s=0.195, max_jitter_s=0.205)

# (folder, split, repeats, slide_frames): testing runs the streaming model, so it needs no shifted copies
for folder, split, repeat, slide in [("training", "train", 2, 10), ("validation", "validation", 1, 10), ("testing", "test", 1, 1)]:
    spectrograms = SpectrogramGeneration(clips=clips, augmenter=augmenter, slide_frames=slide, step_ms=10)
    os.makedirs(os.path.join(args.out_dir, folder), exist_ok=True)  # RaggedMmap only creates the last folder
    RaggedMmap.from_generator(out_dir=os.path.join(args.out_dir, folder, "wakeword_mmap"),
                              sample_generator=spectrograms.spectrogram_generator(split=split, repeat=repeat),
                              batch_size=100, verbose=True)
