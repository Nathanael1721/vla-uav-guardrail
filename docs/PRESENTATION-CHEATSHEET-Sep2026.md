# Presentation Cheatsheet — September 2026

Catatan pribadi untuk presentasi. Bahasa penjelasan Indonesia; kalimat dalam kutipan `"..."` adalah baris yang disarankan untuk diucapkan dalam bahasa Inggris (audiens ITRI internasional). Angka diambil langsung dari `docs/data/eval_sep2026.json` per 2026-09-14 — kalau ada yang bertanya sumbernya, itu jawabannya.

**Jangan lupa duluan:** dua klaim di bawah ini SUDAH DIKOREKSI minggu lalu. Kalau reflek Anda masih mengingat versi lama dari sesi-sesi sebelumnya, jangan diucapkan:

| ❌ JANGAN bilang (versi lama, salah) | ✅ Yang benar sekarang |
|---|---|
| "Ring 10 m salah 37 meter posisinya" | Estimasi jarak ke orang yang **diikuti** sebenarnya benar (43.3 m vs 49.4 m). Yang 37 m itu dibandingkan dengan orang yang salah (bystander terdekat, bukan subjek). |
| "P0 escape rate 0.0 di semua penerbangan" | 0.0 di **41 penerbangan berperisai**. 5 penerbangan kontrol tanpa perisai memang sengaja dibuat gagal (0.63) — itu bukti perisainya perlu. |

---

## BAGIAN A — Mid-Evaluation Deck (17 slide, ~15 menit)
`docs/Guardrail-MidEvaluation-Sep2026.pptx` — untuk sharing meeting 18 September dengan semua dosen partner ITRI.

Alokasi waktu kasar: slide 1–4 (~3 menit, konteks), 5–9 (~5 menit, WP & KPI inti — ini yang terpenting), 10–13 (~4 menit, bukti & keterbatasan — jangan diburu-buru, ini yang membangun kepercayaan), 14–17 (~3 menit, timeline & penutup).

### 1. Cover
Cukup dibaca judulnya. Isi kolom Advisor/Presenter di PPTX sebelum tampil (sengaja dikosongkan sistem).

### 2. The Problem
`"A vision-language model turns a phrase into velocity. Nothing in that model knows what a no-fly zone is. Guardrail sits between the model and the aircraft — a declarative policy enforced on every command."`
Satu kalimat kunci untuk ditutup: **hard KPI: P0 violation escape rate = 0.**

### 3. Work Packages
Tabel WP1–4. Yang perlu ditekankan lisan: **WP3 sudah KPI-grade** (5 run canonical-HIL) — ini satu-satunya baris yang statusnya bukan sekadar "Built".

### 4. Architecture
Alur: Camera → Detector+lock → Estimator → **Safety Shield** (disorot teal) → MAVROS 2 → ArduPilot. Baris bawah: Policy DSL dan Action source **swappable** — tekankan ini kalau ditanya soal ganti model VLA. `"The Shield only sees the Action4D command. It doesn't know or care which model produced it — that's the whole safety argument."`

### 5. Six Constraint Types
Baca 6 kartu sekilas saja (Polygon fence, Altitude, Kinematic, Clearance, Subject stand-off, Corridor). Detail teknis tidak perlu dijelaskan satu-satu kecuali ditanya.

### 6. How the Shield Decides
Alur 5 langkah: Predict → Check → **Repair** (disorot) → Re-check → Fall back. Angka: fail-safe correctness **1.0**, mean repair **4.07 m/s**.

### 7. Where It Runs — PENTING, JANGAN DILEWATI
Tiga rail simulasi. Poin paling jujur di seluruh deck:
`"Tracking runs on Project AirSim — that's 34 flights, camera on, but NOT KPI-grade. The contractual numbers come from ArduPilot SITL over MAVROS 2 — only 5 flights, no camera at all. That's the gap we're closing next."`
Ini menjawab duluan pertanyaan yang biasanya muncul: "kalau KPI-grade tidak pakai kamera, buktinya dari mana?"

### 8. KPI Results (canonical HIL)
Tabel inti. Baca angka P0 escape rate baris pertama: **0.0 / 0.0 / 0.0** untuk NFZ, dynamic NFZ, pedestrian — vs kontrol tanpa perisai **0.63**. Sebutkan sekali: pedestrian di rail ini posisinya **declared**, bukan terdeteksi kamera (SITL tidak punya kamera).

### 9. Guardrail On vs Off
Dua trajectory plot — visual paling kuat di deck. `"Same mission. Shield off goes straight through the no-fly zone — 3.7 seconds inside. Shield on skirts it, zero seconds inside, still reaches the goal."` Untuk pedestrian: **7.07 m → 14.95 m** jarak terdekat.

### 10. What the Camera Rail Shows
Fokus ke baris pertama saja kalau waktu sempit: instance lock **7.1 → 1.9 px** (lebih baik dari null 4.9 px). Baris terakhir penting untuk kejujuran: **1 dari 34** penerbangan kamera memenuhi kedua rate gate (loop ≥9.5 Hz, detector ≥4.0 Hz).

### 11. Class-Conditional Safety — INI JAWABAN UNTUK PERTANYAAN PROF. LAI
Ini slide yang menjawab langsung tantangan Prof. Lai 2 September ("BoT-SORT/YOLO sudah bisa tracking, apa bedanya pakai ViT?").
`"A conventional tracker returns an ID, never a class. It can't tell the Shield which rule to apply. Here, changing one phrase mid-flight — 'a yellow car' to 'a person' — changes the class, and the enforced stand-off ring moves from 5 to 10 meters automatically. Same aircraft, same policy, same hash."`
Kalau ditanya lebih lanjut soal ini, itu **pertanyaan yang paling penting untuk dijawab dengan tenang** — jangan terburu ke slide berikutnya.

### 12. Evidence Discipline
Angka cepat: **460/460** tes cepat, **17/17** coverage, **27** dokumen temuan, **12/13** skenario sweep lolos.

### 13. Honest Limits — JANGAN DILOMPATI, INI YANG MEMBANGUN KREDIBILITAS
6 kartu keterbatasan. Yang paling substansial untuk diucapkan lisan (jangan cuma dibaca dari slide):
`"498 of 516 close-range ticks were people outside the camera's view — bystanders the current rule doesn't protect. And the pedestrian detector still doesn't beat a centre-constant null on that phase."`
Ini kejujuran yang dihargai reviewer akademik — jangan buru-buru lewat.

### 14. Timeline
Sekadar orientasi visual Juni→September. Tidak perlu dibaca semua titik, cukup: `"From a working prototype in July to a KPI-grade contractual gate by late August."`

### 15. Plan to Completion
5 langkah berikutnya. Baca cepat, tekankan #1 (AirSim imagery ke rail KPI) sebagai prioritas tertinggi.

### 16–17. Conclusion + Thank You
`"All five KPIs measured on the canonical rail, P0 escape rate 0.0. Perception is the open half — that's where we're headed next."`

---

## BAGIAN B — Updates Deck (11 slide, ~10 menit)
`docs/Guardrail-Updates-Sep2026.pptx` — kalau audiensnya sudah tahu konteks proyek dan hanya perlu tahu apa yang berubah sejak 2 September.

### 1. Cover
### 2. Action Items
Tabel 7 item dari rapat 2 Sept. Yang **Done**: skenario retarget, diagram arsitektur. Yang **Built, not flown**: CityLife (16 pejalan kaki, 8 mobil — *sudah di-commit ke repo sejak slide ini dibuat, boleh disebut lisan*: `"since committed, still not flown"`). Yang **Open**: Gazebo, sensor real.

### 3. One Phrase, Two Rules
Sama seperti mid-eval slide 11 — dua frame nyata dari penerbangan (mobil t=22.5s, orang t=60.0s p=0.037).

### 4. Four Silent Defects
4 kartu: aturan tak pernah menyala (nama kelas beda), set-point basi, metrik yang bisa dilewati konstanta, verifier yang cuma cek 5 dari 7 field. Semua sudah punya regression test.

### 5. Tracking Steadied
Tabel before/after (jendela waktu yang sama, ≤70s): yaw median **11.5 → 1.3 deg/s**, puncak **130.4 → 9.6 deg/s**. `"Pointing is fixed. Seeing is not — the detector still doesn't beat the null."`

### 6. What the Zero Does Not Cover
Empat angka besar: escape rate **0.0**, **516** tick orang dalam 10m, **498** di antaranya di luar pandangan kamera, bystander terdekat **4.70 m**. Ada footnote retraksi di slide ini — kalau ditanya, itu koreksi yang sudah dijelaskan di atas.

### 7. Why Pedestrians Are Hard
Tabel ukuran subjek vs biaya detektor. Poin fisika: satu patch OWL-ViT = 32px, orang di 16m cuma isi **0.48 patch**. `"768×432 costs nothing offline, but hasn't been flown yet — under Unreal's GPU contention it could be different."`

### 8. CityLife and the Release
16 jalan, 8 mobil; rilis v0.5.0 publik + Pages; 460 tes; satu file angka untuk semua deliverable.

### 9. Next Steps
5 langkah, sama dengan mid-eval slide 15 tapi diurut sedikit beda: terbangkan 768×432 & CityLife dulu, lalu FOV, bystander, Gazebo, kirim koreksi.

### 10–11. Conclusion + Thank You

---

## Pertanyaan yang mungkin muncul (siap-siap)

**Q: "Kenapa P0 escape rate 0.0 tapi orang bisa sampai 4.7 m?"**
`"Because that person was a bystander, not the subject being followed. The rule protects the subject only, by design — extending it to every pedestrian and widening sensor coverage is exactly the next open item."`

**Q: "Kalau tracking bukan KPI-grade, KPI kalian ngukur apa?"**
`"The five contractual KPIs are measured on ArduPilot SITL with a stub pilot and no camera — that rail is what's contractually gated. The camera-based tracking work is functional-rail evidence for the perception half of the system, reported honestly as not-yet-contractual."`

**Q: "Kenapa OWL-ViT bukan Grounding DINO kalau skornya lebih tinggi?"**
`"Grounding DINO scores higher but runs at roughly a fifth the rate — 2.14 Hz ceiling vs 9.95 for OWL-ViT — and our in-flight rate is already near the 4 Hz gate."`

**Q: "Apa langkah paling penting berikutnya?"**
`"Feeding AirSim camera imagery into a Guardrail driven over MAVROS 2, so the tracking evidence becomes contractual instead of a separate rail."`
