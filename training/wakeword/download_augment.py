"""Downloads room impulse responses and background noise for wake-word augmentation, as 16 kHz mono wavs.
Same sources as microWakeWord's training notebook. AudioSet and FMA clips carry mixed licences: personal, non-commercial use."""
import io
import os
import sys
import zipfile

import librosa
import pyarrow.parquet as pq
import soundfile as sf
from huggingface_hub import hf_hub_download, snapshot_download

out = sys.argv[1]


def save_16k(source, dest):
    audio, _ = librosa.load(source, sr=16000, mono=True)
    sf.write(dest, audio, 16000, subtype="PCM_16")


# MIT room impulse responses are already 16 kHz wavs
snapshot_download("davidscripka/MIT_environmental_impulse_responses", repo_type="dataset",
                  allow_patterns="16khz/*.wav", local_dir=out)
os.rename(os.path.join(out, "16khz"), os.path.join(out, "mit_rirs"))

# One AudioSet balanced-train shard (~690 MB, ~2k clips of everyday sounds)
os.makedirs(os.path.join(out, "audioset_16k"))
shard = hf_hub_download("agkphysics/AudioSet", "data/bal_train/00.parquet", repo_type="dataset")
for b, batch in enumerate(pq.ParquetFile(shard).iter_batches(batch_size=64, columns=["audio"])):
    for i, row in enumerate(batch.column("audio").to_pylist()):
        name = os.path.splitext(os.path.basename(row["path"] or f"{b}_{i}"))[0]
        save_16k(io.BytesIO(row["bytes"]), os.path.join(out, "audioset_16k", name + ".wav"))

# Free Music Archive extra-small set (music in the background)
os.makedirs(os.path.join(out, "fma_16k"))
with zipfile.ZipFile(hf_hub_download("mchl914/fma_xsmall", "fma_xs.zip", repo_type="dataset")) as z:
    for name in z.namelist():
        if name.endswith(".mp3"):
            save_16k(io.BytesIO(z.read(name)), os.path.join(out, "fma_16k", os.path.basename(name)[:-4] + ".wav"))
