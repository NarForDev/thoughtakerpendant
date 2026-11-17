# Wake-word training

Trains the pendant's three wake-word models ("Perthro", "Perthro mark", "Perthro forget") with
[microWakeWord](https://github.com/kahrendt/microWakeWord) (commit 4665173). Runs in WSL2 Ubuntu-24.04 under `/root/mww`;
generated audio, features and models stay there, not in git.

## Setup (once)

```sh
cd /mnt/d/NAR/Coding/thoughtakerpendant
source /root/mww/venv/bin/activate            # activate also puts nvidia cusolver on LD_LIBRARY_PATH for TF
export PYTHONPATH=/root/mww/piper-sample-generator
python training/wakeword/download_augment.py /root/mww/augment
```

Negative features: `dinner_party`, `dinner_party_eval`, `no_speech`, `speech` from
https://huggingface.co/datasets/kahrendt/microwakeword, unzipped into `/root/mww/negatives`.

## Per phrase

| P | TEXT | NEAR (near-misses, trained as negatives) |
|---|---|---|
| perthro | `Perthro` | `per throw` `birth row` `Pedro` `Perth` `partner` `further` `pyro` `porch row` |
| perthro_mark | `Perthro mark` | `Perthro` `mark` `mark that` `Perthro park` `Perthro march` `market` `Perthro forget` |
| perthro_forget | `Perthro forget` | `Perthro` `forget` `forget it` `Perthro forgive` `Perthro for good` `Perthro mark` |

```sh
P=perthro; TEXT="Perthro"; NEAR=("per throw" "birth row" "Pedro" "Perth" "partner" "further" "pyro" "porch row")
W=/root/mww/work/$P
GEN=(python -m piper_sample_generator --model /root/mww/piper-sample-generator/models/en_US-libritts_r-medium.pt --batch-size 100)

"${GEN[@]}" "$TEXT" --max-samples 5000 --noise-scales 0.667 0.98 --noise-scale-ws 0.8 0.98 --length-scales 0.8 1.0 1.2 --output-dir $W/positive_wav
for n in "${NEAR[@]}"; do "${GEN[@]}" "$n" --max-samples 300 --output-dir "$W/adversarial_wav/${n// /_}"; done

python training/wakeword/make_features.py $W/positive_wav $W/positive_features
python training/wakeword/make_features.py $W/adversarial_wav $W/adversarial_features

python -m microwakeword.model_train_eval --training_config training/wakeword/$P.yaml \
  --train 1 --restore_checkpoint 1 --test_tflite_streaming_quantized 1 --use_weights best_weights \
  mixednet --pointwise_filters "64,64,64,64" --repeat_in_block "1,1,1,1" \
  --mixconv_kernel_sizes '[5], [7,11], [9,15], [23]' --residual_connection "0,0,0,0" \
  --first_conv_filters 32 --first_conv_kernel_size 5 --stride 3
```

Result: `$W/model/tflite_stream_state_internal_quant/stream_state_internal_quant.tflite`, with recall and false accepts per hour
in the logs.
