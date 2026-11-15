# Per-paraphrase robustness on hil, 15 Nov 2026

The PAS follow mission (AIoT-VLA v10b on the AGX Orin, Project AirSim CityLife + ArduPilot SITL 4.5.7, `--identity --negative-query "a traffic light" --target-estimator`) flown once per paraphrase: the source "follow a red car" and the paraphraser's 8 stored paraphrases of it (guardrail/paraphraser.py, store set `follow a red car`), each given as the VLA's grounding phrase (`--vla-object`); the detector's start-gate query stays "a red car". Outcome = how the mission ended.

The source, flown twice: target_lost, target_lost. The model's own noun phrase "a red car" (the default every other AIoT-VLA flight used), flown as a control: held to max_s. Of the 8 paraphrases, 2 never started (the start gate, whose detector query is the fixed "a red car" and reads no paraphrase, found no car within 30 m in 330 s); of the 6 that started, **4/6 ended as the source did** and 2/6 held the car to max_s. P0 escape ticks over all 11 runs: 0. One flight per paraphrase.

| run | paraphrase | text | KPI-grade | ended | ticks (Hz) | interventions | P0 escape ticks | estimate within 6 m |
|---|---|---|---|---|---|---|---|---|
| pas_hil_para_00 | source | follow a red car | YES | target_lost | 899 (9.974) | 0 | 0 | 0.0133 |
| pas_hil_para_01 | pp-ddaa2423665c7006 | track a red car | YES | max_s reached | 4186 (9.965) | 234 | 0 | 0.828 |
| pas_hil_para_02 | pp-3ae8114f0b2c8f02 | keep up with a red car | YES | target_lost | 899 (9.982) | 0 | 0 | 0.0111 |
| pas_hil_para_03 | pp-7c3e1816a4183abe | pursue a red automobile through the streets | no | no mission: start gate timed out after 330.1 s: no plausible 'a red car' within 30 m on 5 inferences in a row; landing | 0 (None) | 0 | 0 | None |
| pas_hil_para_04 | pp-266a00cb70c06e1e | tail a red car as it drives | YES | target_lost | 900 (9.984) | 0 | 0 | 0.0144 |
| pas_hil_para_05 | pp-2e8ac2cc00830ae6 | Please follow a red car. | YES | target_lost | 900 (9.98) | 0 | 0 | 0.0089 |
| pas_hil_para_06 | pp-81ab2824ae47421c | Could you shadow a red car for me? | YES | target_lost | 2077 (9.982) | 81 | 0 | 0.5542 |
| pas_hil_para_07 | pp-5bf613700c32e8b3 | There's a red car - I'd like you to follow it. | no | no mission: start gate timed out after 330.1 s: no plausible 'a red car' within 30 m on 5 inferences in a row; landing | 0 (None) | 0 | 0 | None |
| pas_hil_para_08 | pp-0e58648d12901083 | Your task is to keep tracking a red car wherever it goes. | YES | max_s reached | 4184 (9.958) | 206 | 0 | 0.8994 |
| pas_hil_para_09 | noun-phrase | a red car | YES | max_s reached | 4186 (9.964) | 244 | 0 | 0.9157 |
| pas_hil_para_10 | source-repeat | follow a red car | YES | target_lost | 900 (9.98) | 4 | 0 | 0.0133 |
