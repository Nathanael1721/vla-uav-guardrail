# Meeting Pack — September 2026

Companion to `docs/VLA-Guardrail-Sept2026.pptx` (14 slides). Bilingual:
**English first, Bahasa Indonesia below each block.**

Five parts:
**A** answers to the 19 August items · **B** per-slide script ·
**C** the 60-second story · **D** Q&A bank · **E** numbers to memorise

Everything here was verified against the repository on 2026-09-01. Where an item
is **not** done, it says so — those are the ones that get asked about.

> **Dua bahasa.** Bagian Inggris untuk dibaca saat presentasi; bagian Indonesia
> untuk Anda pahami dan hafalkan. Jangan menerjemahkan angka — angka sama di
> kedua bahasa.

---

# Part A — Answers to the 19 August review

## A1. The four action items from the minutes

| # | What was asked | Status | Evidence |
|---|---|---|---|
| 1 | Nathan: write and submit the midterm report + demo video | **Done** | `docs/MIDTERM-REPORT-Aug2026.pdf`, `docs/VLA-Guardrail-Midterm-Aug2026.pptx` |
| 2 | Nathan: add a pedestrian 3D model; extend to multi-object tracking | **Done, and flown** | 12 pedestrians + 12 parked vehicles + moving traffic; `demo/out/city_locked` |
| 3 | Nathan: evaluate and attempt ArduPilot–MAVLink SITL with ROS | **Done, beyond what was asked** | MAVROS 2 on ROS 2 Jazzy; `canonical-hil` topology; KPI-grade runs |
| 4 | Speaker 1: confirm phase-1 disbursement | Not ours | — |

**Say it like this.** *"All three of my action items are closed. The SITL one
went further than the minutes asked: it is not just running, it is the topology
the grant's KPI gate requires, so the numbers I am about to show are contractual
numbers rather than demo numbers."*

> **Indonesia.** Ketiga tugas saya selesai. Yang SITL melampaui permintaan
> notulen: bukan sekadar jalan, tapi sudah memakai topologi yang disyaratkan
> gate KPI dalam kontrak — jadi angka yang saya tunjukkan adalah angka
> kontraktual, bukan angka demo.

## A2. The five AI-flagged open items

**1 — Root cause of the resolution/latency trade-off.**
**Measured and split into two costs.** With the recorder writing *nothing at
all*, the loop still only reached 8.16 Hz — so about **1.8 Hz of the loss is the
Chase camera's render and deserialisation**, GPU-side, exactly as suspected in
the meeting. The rest was Python doing JPEG encoding on the control loop; that
half is fixed (recording moved to its own thread).
**But be honest:** the pre-registered gate was loop ≥ 9.5 Hz and detector
≥ 4.0 Hz, and **0 of 6 recorded camera runs meet it** — loop 7.45–8.03 Hz,
detector 3.68–5.15 Hz. The 1280×720 rung was formally rejected and the config
backed off to 960×540, but the remaining rungs of the ladder were never recorded.
This does **not** touch the KPI numbers: the canonical rail has no camera.

**2 — Depth camera vs LiDAR.**
Still undecided, and our position is that it is not yet the binding problem.
Depth arrives as `16UC1` **quantised to whole metres** — fine for "is something
close", useless for "hold 10.0 m". The cheaper first step is putting the missing
obstacles into the map we already have. **And that step is now overdue:** the
parked vehicles and pedestrians added on 31 August are *not* in
`occ_day.npz`, which was last built on 25 August. They are visible to the camera
and invisible to `ObstacleClearance`.

**3 — Multi-object / pedestrian policy rules undefined.**
**Closed.** `SubjectStandoff` is a real constraint type with a `subject_class`
field: 10 m for a pedestrian, 5 m for anything else. It is hashed into
`policy_hash`, written to the audit log, and enforced by the Shield — not a
command-line flag. Flown on the canonical rail: **shield off, closest approach
7.07 m and 2.3 s inside the ring; shield on, 14.95 m and 0.0 s.**

**4 — No concrete route for ViT↔VLA fusion.**
The interface already exists — the `Action4D` contract; five different action
sources have flown behind the same Shield. The fusion the minutes proposed is
built (`demo/vla_bridge.py`) and **measured as worse**: AerialVLA in-loop runs at
0.11 Hz, about 9 seconds per decision, during which a 2 m/s car travels 5.4 m.
**The proposal for this meeting is the two-rate design:** VLA at ~0.4 Hz for the
semantic decision, `servo()` at ~5 Hz for tracking, Shield last.

**5 — Retraining YOLO/COCO for colour.**
**Recommend declining**, and this period we measured why. OWL-ViT is already
open-vocabulary, so there is nothing to retrain to make "yellow car"
expressible. Colour is handled by the chroma gate. We surveyed the alternatives
properly: Grounding DINO scores our pedestrians **5.2× better** than OWL-ViT
(0.327 vs 0.062) — and **none of the alternatives clears the 4.0 Hz gate**
(2.1 Hz measured; ~3.5 Hz at best after tuning). YOLO-World, which I proposed in
August as the speed answer, turns out to ship under **AGPL-3.0** — whether that
is acceptable in an ITRI subcontract is a legal question, not a technical one,
and I would like a decision on it rather than making one quietly.

> **Indonesia — ringkasan lima item.** (1) Penyebab latensi **sudah terukur**:
> ~1,8 Hz dari render kamera, sisanya JPEG di loop kontrol (sudah diperbaiki) —
> **tapi jujur: gate 9,5 Hz masih belum tercapai, 0 dari 6 run**. Ini tidak
> memengaruhi angka KPI karena rail kanonik tidak berkamera. (2) Depth vs LiDAR
> belum diputuskan; depth terkuantisasi 1 meter. **Dan objek yang kami tambahkan
> 31 Agustus belum masuk peta rintangan** — ini utang yang harus saya sebut.
> (3) Aturan jarak per objek **selesai** — `SubjectStandoff`, terbang 14,95 m
> lawan cincin 10 m. (4) Fusion sudah ada dan **terukur lebih buruk**; usulan
> saya desain dua-laju. (5) Retraining warna **tidak disarankan**; Grounding DINO
> 5,2× lebih baik untuk pejalan kaki tapi tak ada yang lolos gate 4 Hz, dan
> YOLO-World berlisensi AGPL — itu keputusan hukum, bukan teknis.

## A3. The follow-ups I set myself — including the two still open

| Item | Status |
|---|---|
| Correct "100M" → **153M** and "MHz" → **Hz** everywhere | **Done** — no wrong figure remains in the deck or report |
| Set `--object-width-m` per class before any pedestrian flight | **Done** — derived per class |
| Per-object stand-off as a hashed rule, not a CLI flag | **Done** |
| Wire `run_sitl_demo.py` to `build_manifest()` + `kpi.py` | **Done** |
| MAVROS 2, then open the `canonical-hil` guard | **Done** |
| Midterm report stating the failed gate honestly | **Done** |
| Run the full 960×540 back-off ladder and record what was kept | **Partial** — the failing rung is recorded and the config was backed off; the comparison rungs were never run |
| Add trees / street furniture / parked vehicles to `occ_day.npz` | **Done, 25 Aug** — see the correction below |

> **Correction, 2026-09-07.** This row said "still open — and now wider" and that
> was wrong. `occ_day.npz` was rebuilt over the 6–14 m flight band on **25
> August**, and the cell holding the documented 9 m collision at (48.3, −0.9) is
> occupied in it. I carried the claim forward from the 19 August minutes without
> testing it against the map, which takes one command. Do not repeat it in the
> meeting.
>
> What is genuinely undecided is narrower: `follow_pedestrian.yaml` permits
> descent to **4 m** while loading the 6–14 m band map, and `ground_2to4.npz`
> exists for that altitude with nothing selecting it.

**Do not hide the back-off ladder row.** Saying it first is what makes the rest
credible.

> **Indonesia.** Enam dari delapan selesai. Dua belum: tangga back-off resolusi
> baru sebagian, dan objek jalanan belum masuk peta rintangan. **Sebutkan dua ini
> lebih dulu** — justru itu yang membuat sisanya dipercaya.

---

# Part B — Per-slide script (14 slides, ~15 minutes)

Each slide: **hook** (one line to open with) → **say** (the paragraph) →
**hold in reserve** (only if asked).

---

### Slide 1 · Cover
**Hook.** "Progress since the midterm report — and one thing we found that the
midterm report could not have found."
**Say.** Name, the grant title, the reporting period. Move on quickly.
> **ID.** "Perkembangan sejak laporan midterm — dan satu temuan yang tidak
> mungkin ditemukan oleh laporan midterm."

### Slide 2 · The hard KPI is measured, not inferred
**Hook.** "The headline number did not change. What changed is that it is now a
measurement."
**Say.** The P0 escape rate was previously *inferred* — from whether the Shield
had acted at all. Every code branch that raises a violation also appends a
repair, so that test could never return non-zero. It could not detect the case
the KPI exists to catch: a Shield that repairs an illegal action into a
different illegal action. The Shield now re-checks the action it actually flew
and records the result. Re-checked by hand, the old numbers were right — but the
measurement was not.
**Reserve.** `p0_ticks_not_measurable` is 0 on all canonical runs; older logs
that predate the field are counted separately rather than assumed clean.
> **ID.** Angka utama tidak berubah; yang berubah, sekarang ia hasil
> **pengukuran**, bukan kesimpulan. Dulu escape rate disimpulkan dari "apakah
> Shield bertindak" — dan setiap cabang yang menaikkan pelanggaran juga menambah
> repair, jadi tes itu **tidak mungkin** bernilai bukan-nol.

### Slide 3 · "Hold 10 m from a person", flown
**Hook.** "This is Speaker 1's question from August, answered on the rail that
counts."
**Say.** The request was a WP1 gap: the schema could not express "10 m from a
person, different policies for different objects". It can now. Shield off, the
aircraft came within **7.07 m** and spent 2.3 s inside the ring. Shield on,
closest approach **14.95 m**, zero seconds inside. Both runs still reached the
target — the rule constrains without cancelling the mission.
**Reserve.** The subject position is *declared*, not perceived, because this rail
has no camera. That is deliberate: the review confirmed the Shield is the
deliverable and the pilot is a swappable input.
> **ID.** Permintaan Speaker 1 Agustus lalu, dijawab di rail yang dihitung
> kontrak. Tanpa Shield 7,07 m; dengan Shield 14,95 m dan nol detik di dalam
> cincin. Kedua-duanya tetap sampai tujuan.

### Slide 4 · A metric that measured the wrong thing
**Hook.** "I need to correct something I reported."
**Say.** `det_hit_rate` counts inferences that produced *any* box. A box on a
parked look-alike scores exactly like a box on the target. It is a
detector-liveness rate, not a tracking-accuracy rate. Re-measured properly, the
flight with the best reported rate — 0.995 — was tracking the wrong vehicle for
about a quarter of its mission, including frames where the target was outside
the camera entirely.
**Reserve.** `frac_on_target` scores against ground truth that was already in
every flight log; no re-flying was needed to discover this.
> **ID.** Saya perlu mengoreksi sesuatu. `det_hit_rate` hanya menghitung apakah
> detektor menghasilkan kotak — kotak di mobil mirip yang terparkir dinilai sama
> dengan kotak di target. Penerbangan dengan angka terbaik justru melacak
> kendaraan yang salah selama ~24% misi.

### Slide 5 · Instance lock
**Hook.** "The fix was two numbers, and both came from measurement."
**Say.** Real tracking moves the box a median of 0.0 px per tick, so the old
gate of 0.28 of image width admitted thirteen times more motion than tracking
ever produces — and passed the two jumps onto the wrong vehicle. Tightened to
0.12. Then size: the target measured 20–24 px wide and every wrong box 47–84 px,
so anything more than 1.8× different in width is a different object. Result:
**every box on the correct vehicle**, p95 error 24.5 px, zero off-frame
detections.
> **ID.** Gate lama 0,28 lebar citra; pelacakan nyata menggeser kotak median
> 0,0 px. Diperketat ke 0,12, ditambah uji konsistensi ukuran 1,8×. Hasil:
> seluruh kotak pada kendaraan yang benar.

### Slide 6 · Scene enrichment
**Hook.** "You asked for a street that looks real. The affordable choice and the
realistic choice turned out to be the same choice."
**Say.** 12 parked vehicles, 12 pedestrians, plus moving traffic. A moving
object costs one pose update per control tick, inside the same loop that runs
the detector. A real street carries far more parked vehicles than moving ones,
so most of the scene is static — and static costs nothing per tick.
> **ID.** Objek bergerak memakan satu update pose per tick kontrol, di loop yang
> sama dengan detektor. Jalan sungguhan lebih banyak kendaraan parkir daripada
> yang bergerak — jadi pilihan realistis dan pilihan murah kebetulan sama.

### Slide 7 · Five of five KPIs now measured ★
**Hook.** "The grant names five acceptance KPIs. Until this period we computed
three."
**Say.** Mean repair magnitude and mean time to safe had no number anywhere — not
in a report, not in a deck — although every field they need has been in the
flight logs since the first flight. Both are now measured. Shield on: mean repair
**4.07 m/s**, max 7.21, and **zero unsafe-position episodes**. Shield off: 4.1
seconds to get out of a place it should not have been. All 42 delivered runs were
rescored without re-flying anything, and every previously published P0 figure
reproduced exactly.
**Reserve.** Time to safe is measured from the **position** being illegal, not
the action. Measured off actions it reports 21.9 s for a run whose real unsafe
dwell was 0.0 — the aircraft flew alongside a no-fly zone while the Shield
trimmed a pilot that kept turning into it. That is the Shield working.
> **ID.** ★ Slide terpenting. Kontrak menyebut lima KPI; selama ini kami hitung
> tiga. Dua sisanya kini terukur — dari data yang **sudah ada** di log sejak
> penerbangan pertama. Shield nyala: magnitudo repair 4,07 m/s, **nol episode
> posisi tidak aman**. Shield mati: 4,1 detik untuk keluar.

### Slide 8 · A recovery that never arrived ★★
**Hook.** "And measuring it correctly found a real defect within an hour."
**Say.** The altitude repair aimed the recovery climb at the floor *exactly*,
which makes it a decaying exponential that converges on the boundary without
crossing it. From 3 m against a 10 m floor: 9.10 m at six seconds, 9.88 m at
twelve, **9.99973 m after thirty seconds** — and it would stay below for any
length of flight. Every existing KPI called this healthy, and each was right to:
the emitted action climbs, so it is legal on every tick, and the escape rate
stayed zero the whole way down. **The action was always fine. The state never
became safe.** Fixed: recoveries now aim a margin inside the band and arrive in
about six seconds.
**Reserve.** The three KPIs we had are all properties of *actions*. Time to safe
is the first that is a property of the *trajectory*. That is the argument for
having measured it.
> **ID.** ★★ Ini kartu terkuat Anda. Perbaikan altitude membidik **tepat** di
> batas, jadi pendakiannya eksponensial meluruh yang tak pernah menyeberang:
> 9,99973 m setelah 30 detik, selamanya di bawah lantai. Semua KPI lama
> menyebutnya sehat — dan **benar**, karena aksinya legal tiap tick.
> **Aksinya selalu benar; state-nya tidak pernah aman.**

### Slide 9 · Two rule classes the DSL could not express
**Hook.** "Both are in the grant's DSL specification. Both were missing from our
code — and from the reference implementation as well."
**Say.** `corridor` is the first keep-**IN** rule: a fence says where you may not
go, a corridor says the route is the only place you may. `valid_time` is a
recurring schedule — a weekday school curfew expressed on the fence itself. Six
constraint classes implemented now, against two in the reference implementation's
own policy-DSL package.
**Reserve.** One detail I would have got wrong without reading the spec: a time
window is a **field any rule may carry**, not a constraint class of its own.
> **ID.** `corridor` adalah aturan keep-IN pertama; `valid_time` adalah jadwal
> berulang. Enam kelas aturan sekarang, dibanding dua di paket policy-DSL repo
> Prof. Satu detail yang nyaris salah: jendela waktu itu **field**, bukan tipe.

### Slide 10 · The sweep harness the report said was not built
**Hook.** "The midterm report says, in those words, 'scenario sweep harness not
built'."
**Say.** Twelve scenarios, run headless — Shield plus a kinematic integrator, no
simulator, no GPU, no autopilot — in about a second. That is the difference
between a harness that runs on every change and one that runs when somebody
remembers. Eleven pass. One flies **unshielded** and is gated to *fail* on
safety: if an unguarded run into a no-fly zone ever scores clean, every other
pass is worthless.
**Reserve.** `standoff-wedge` is recorded as a known failure, on purpose — see
the next question in the Q&A.
> **ID.** Laporan midterm menulis apa adanya: "scenario sweep harness not built".
> Sekarang 12 skenario, headless, ~1 detik. Satu skenario sengaja terbang **tanpa
> Shield** dan wajib GAGAL uji keselamatan — kalau penerbangan tanpa pengaman
> bisa lolos bersih, semua kelulusan lain tak ada artinya.

### Slide 11 · Three named deliverables, now produced
**Hook.** "Three things the grant names and we had never produced."
**Say.** A signed policy bundle — reloading refuses a tampered IR, a foreign
manifest, or a truncated archive. WGS84 authoring, additive, so every existing
metre policy still loads unchanged. And the Constraint Summary Pack, which
exposed a real gap: the prompt handed to the pilot listed fences, altitude and
speed — and nothing else. On the pedestrian policy it described none of the 10 m
stand-off the Shield enforces against it.
**Reserve.** The signature is an honest placeholder, and so is the reference
implementation's. See the Q&A.
> **ID.** Bundle kebijakan bertanda tangan, WGS84 (aditif), dan Constraint
> Summary Pack — yang justru membongkar celah nyata: prompt ke pilot tidak pernah
> menyebut aturan standoff 10 m yang ditegakkan Shield terhadapnya.

### Slide 12 · Remaining work
**Hook.** "Ordered by what it contributes to acceptance, not by what is easy."
**Say.** Perception on the KPI-grade rail is still the largest item: ArduPilot
SITL has no renderer, so every tracking result sits outside the contractual gate.
Then the rate gate the camera rail still misses. Then the scenery that is not in
the obstacle map. Then replay bundles, the wedge, and the frame mismatch with the
reference implementation.
> **ID.** Diurutkan menurut kontribusi ke penerimaan kontrak, bukan menurut
> kemudahan. Yang terbesar tetap perception di rail KPI.

### Slide 13 · Deliverables
**Hook.** "Everything on these slides is reproducible from the repository."
**Say.** Every figure is read from the artefacts at build time — nothing is typed
onto a slide. Videos ship alongside rather than embedded.
> **ID.** Setiap angka dibaca dari artefak saat build; tak ada yang diketik
> manual ke slide.

### Slide 14 · Closing
**Hook.** "Zero. And now we know what zero was not telling us."
**Say.** P0 escape rate 0, measured on the canonical topology, zero unmeasurable
ticks, across three configurations. All five acceptance KPIs are now computed —
and the two added this period found a defect that a perfect escape rate could
never have shown.
> **ID.** Nol — dan sekarang kami tahu apa yang **tidak** diberitahukan oleh nol
> itu.

---

# Part C — The 60-second story

> "A Vision-Language-Action model flies the drone; because it is a neural
> network, it sometimes asks for things that are not allowed. Our **Guardrail**
> sits between the model and the autopilot and checks every single action
> against hard rules. The headline number is unchanged — **zero P0 safety
> escapes** — but this period it became a *measurement* rather than an
> inference, on the ArduPilot topology the grant actually requires.
>
> Three things are new. Speaker 1's request from August — *hold 10 m from a
> person* — is now a hashed, audited rule, and it flies: **14.95 m against a 10 m
> ring**. All five contractual KPIs are computed for the first time; two of them
> had never had a number. And measuring one of those two found a real defect:
> the altitude recovery converged on the floor without ever crossing it —
> 9.99973 metres after thirty seconds — while every existing KPI correctly
> reported the system as healthy, because the *action* was legal every tick and
> only the *state* was wrong.
>
> What is still open, plainly: all the camera-based tracking evidence sits
> outside the contractual gate, because ArduPilot SITL has no renderer. That is
> the next piece of work."

> **Indonesia.** "Model VLA menerbangkan drone; karena ia jaringan saraf, kadang
> ia meminta hal yang tidak diizinkan. **Guardrail** kami duduk di antara model
> dan autopilot, memeriksa setiap aksi terhadap aturan keras. Angka utama tidak
> berubah — **nol pelanggaran P0** — tapi periode ini ia menjadi **hasil
> pengukuran**, bukan kesimpulan, di topologi ArduPilot yang disyaratkan kontrak.
>
> Tiga hal baru. Permintaan Speaker 1 — *jaga 10 m dari orang* — kini jadi aturan
> yang di-hash dan diaudit, dan terbang: **14,95 m lawan cincin 10 m**. Kelima
> KPI kontrak dihitung untuk pertama kalinya; dua di antaranya belum pernah punya
> angka. Dan mengukur salah satunya menemukan cacat nyata: pemulihan altitude
> mendekati lantai tanpa pernah menyeberanginya — 9,99973 meter setelah 30 detik
> — sementara semua KPI lama dengan benar melaporkan sistem sehat, karena
> **aksinya** legal tiap tick dan hanya **state**-nya yang salah.
>
> Yang masih terbuka, terus terang: seluruh bukti pelacakan berbasis kamera ada
> di luar gate kontrak, karena ArduPilot SITL tidak punya renderer."

---

# Part D — Q&A bank

Ordered by how likely they are, and the uncomfortable ones are included because
those are the ones that get asked.

### D1. "Your escape rate was already zero. What did the two new KPIs buy?"
**Answer.** A defect that zero could not have shown. The three KPIs we had are
all properties of *actions*: did an illegal one fly, was the fail-safe correct,
how often did the Shield intervene. A Shield that repairs every action into
another legal-but-useless one scores perfectly on all three. Mean time to safe is
the first that is a property of the *trajectory*, and within an hour of existing
it found the altitude recovery that never arrived.
> **ID.** Ia membeli sebuah cacat yang tak mungkin ditunjukkan oleh angka nol.
> Tiga KPI lama semuanya sifat **aksi**; yang baru ini sifat **lintasan**. Dalam
> sejam ia menemukan pemulihan altitude yang tak pernah sampai.

### D2. "Why is the perception work outside the KPI gate?"
**Answer.** Because ArduPilot SITL has no renderer. The KPI gate requires the
canonical topology — ArduPilot plus MAVROS 2 — and the camera lives in Project
AirSim. So the tracking numbers are real measurements on a functional rail, but
they are not contractual numbers, and I do not present them as such. Closing it
means feeding AirSim imagery to a Guardrail driven over MAVROS: the
`HIL_GPS`/`HIL_SENSOR` bridge. That is the single largest remaining item.
> **ID.** Karena ArduPilot SITL tidak punya renderer, sedangkan kamera ada di
> AirSim. Angka pelacakan itu pengukuran nyata, tapi bukan angka kontrak — dan
> saya tidak menyajikannya sebagai angka kontrak.

### D3. "You marked a scenario as a known failure. Is the Shield broken?"
**Answer.** No — and the distinction matters. In that scenario the subject stands
exactly on the route, so every heading that makes progress also closes the range.
The Shield holds: no illegal action is ever emitted and the 10 m ring is never
broken. What fails is the *mission* — the aircraft never arrives. That is the gap
between "the action was repaired" and "the trajectory was sensible", and a filter
that can only veto cannot close it; it needs a planner that can route around a
constraint. I chose to record it in the sweep rather than leave it in a document,
so it is reported every run instead of being forgotten.
> **ID.** Tidak. Subjek berdiri **tepat** di garis rute, jadi setiap arah yang
> maju juga memperpendek jarak. Shield tetap benar — tak ada aksi ilegal, cincin
> 10 m tak pernah ditembus. Yang gagal adalah **misinya**. Itu celah antara "aksi
> sudah diperbaiki" dan "lintasan masuk akal".

### D4. "Is the bundle signature real?"
**Answer.** No, and I have not dressed it up as one. It is a placeholder that
binds the policy hash to a signing identity — and the reference implementation's
is the same placeholder, with a comment saying the real CA, lab or ITRI, is an
open question. What the bundle *does* guarantee today is integrity against
accident and silent edits: reloading refuses a tampered IR, a manifest from
another policy, or a truncated archive. It does not defend against an attacker
who can rewrite both, and it does not claim to. Deciding the CA is a question for
you, not for me.
> **ID.** Bukan, dan saya tidak mendandaninya. Itu placeholder — sama dengan yang
> ada di repo Prof, lengkap dengan catatan bahwa CA sesungguhnya masih pertanyaan
> terbuka. Yang dijamin sekarang: integritas terhadap kecelakaan dan penyuntingan
> diam-diam.

### D5. "Why not just use a faster or better detector?"
**Answer.** We measured all of them. Grounding DINO detects our pedestrians
**5.2× better** than OWL-ViT — 0.327 against 0.062 — which is exactly our weakest
area. But **none of the alternatives clears the 4.0 Hz detector gate**: 2.1 Hz as
benchmarked, about 3.5 Hz at best after tuning, and those are ceilings on an idle
GPU. In flight OWL-ViT already measures 4.3× worse than its idle figure because
it shares the machine with Unreal. A model that ceilings at 3.5 Hz idle would run
near 1 Hz in flight, and a stale target position is a control problem, not a
cosmetic one. The design proposal is two-rate: Grounding DINO at a low rate for
acquisition, OWL-ViT per tick for tracking.
> **ID.** Sudah kami ukur semua. Grounding DINO **5,2× lebih baik** untuk pejalan
> kaki — persis kelemahan kami — **tapi tak satu pun lolos gate 4,0 Hz**. Usulan:
> desain dua-laju.

### D6. "What about YOLO-World? You suggested it in August."
**Answer.** I did, and I have to walk part of it back. Technically it is still the
right shape — YOLO speed with an open vocabulary. But `ultralytics` ships under
**AGPL-3.0**, and whether that licence is acceptable in an ITRI subcontract
deliverable is a legal decision, not a technical one. I did not want to make it
quietly, so I excluded it from the benchmark and am raising it here.
> **ID.** Benar, dan sebagian harus saya tarik. Secara teknis masih tepat, tapi
> `ultralytics` berlisensi **AGPL-3.0** — apakah itu bisa diterima dalam
> subkontrak ITRI adalah keputusan hukum, bukan teknis.

### D7. "Did you meet the frame-rate gate you set yourself?"
**Answer.** No. The gate was loop ≥ 9.5 Hz and detector ≥ 4.0 Hz, fixed before
the runs; **zero of six recorded camera runs meet it** — loop 7.45 to 8.03 Hz. I
rejected the 1280×720 configuration on that basis and backed off to 960×540, but
I have not run the rest of the ladder. It does not affect the KPI numbers,
because the canonical rail carries no camera — but it is an open item and I would
rather say so than have it found.
> **ID.** Tidak. Gate-nya loop ≥ 9,5 Hz; **nol dari enam** run memenuhinya. Tidak
> memengaruhi angka KPI karena rail kanonik tanpa kamera — tapi ini item terbuka
> dan lebih baik saya sebut sendiri.

### D8. "Are the parked cars and pedestrians protected by the Shield?"
**Answer.** Partly, and the distinction is important. A pedestrian who is the
*tracked subject* is protected by `SubjectStandoff` — that is the 10 m rule. But
as scenery they are **not** in the occupancy map, which was last built on 25
August, before they were added on the 31st. So `ObstacleClearance` cannot see
them. That is a data gap, not a Shield defect — the Shield cannot protect against
geometry that is not in its map — and closing it needs no new sensor, just a map
rebuild. It is on the open list.
> **ID.** Sebagian, dan bedanya penting. Pejalan kaki yang menjadi **subjek yang
> dilacak** dilindungi `SubjectStandoff`. Tapi sebagai dekorasi mereka **belum**
> masuk peta okupansi. Itu celah data, bukan cacat Shield.

### D9. "When will the HIL bridge be done?"
**Answer.** I would rather not put a date on it in this meeting. It needs WSL,
ArduPilot, and the `HIL_GPS`/`HIL_SENSOR` path, and it is the one remaining item
whose size I genuinely do not know yet. What I can commit to is scoping it
properly and coming back with an estimate — and to the priority order: it is
first on the open list.
> **ID.** Saya lebih baik tidak menyebut tanggal di rapat ini. Yang bisa saya
> janjikan: melakukan scoping dan kembali dengan estimasi — dan urutan
> prioritasnya nomor satu.

### D10. "How do I know the numbers on these slides are real?"
**Answer.** Every figure is read from `demo/out/<tag>/kpi.json` and
`metrics.json` at build time. Nothing is typed onto a slide, so a slide cannot
drift from the data it reports. That rule earned its keep this period: when
`det_hit_rate` turned out to measure the wrong thing, correcting every slide was
a rebuild rather than a hunt.
> **ID.** Setiap angka dibaca dari artefak saat build; tak ada yang diketik. Saat
> `det_hit_rate` ternyata salah ukur, mengoreksi seluruh slide cukup dengan
> membangun ulang.

### D11. "The VLA still is not really flying it, is it?"
**Answer.** Correct, and that is a deliberate design position the review itself
confirmed: the Guardrail is the deliverable and the pilot is a swappable input.
Five different action sources have flown behind the same Shield — a real 7B VLA,
a UAV-tuned LoRA, our own fine-tunes, a behaviour-cloning policy, and a
hand-written controller — with a P0 escape rate of 0 in all five. Five occupants
of one slot is the architectural result, not an excuse.
> **ID.** Benar, dan itu posisi desain yang dikonfirmasi review: **Guardrail
> adalah deliverable-nya**, pilot bisa diganti. Lima sumber aksi berbeda sudah
> terbang di belakang Shield yang sama, escape rate 0 pada kelimanya.

### D12. "Why measure time to safe from position and not from the violation?"
**Answer.** Because a violation means two different things: the requested
*action* is illegal, or the *position* is. Only the second is what "time to safe"
asks about. Measured off actions, one run reports 21.9 seconds while its
independently computed unsafe dwell is 0.0 — the aircraft flew alongside a no-fly
zone for twenty seconds while the Shield trimmed a pilot that kept turning into
it. Reporting that as "it took 21.9 seconds to become safe" would be false. We
already made a naming mistake like that once this period, with `det_hit_rate`,
and I did not want to make it twice.
> **ID.** Karena pelanggaran punya dua arti: **aksi** yang ilegal, atau **posisi**
> yang ilegal. Hanya yang kedua ditanyakan "time to safe". Diukur dari aksi, satu
> run melaporkan 21,9 detik padahal dwell tak-amannya 0,0.

---

# Part E — Numbers to memorise

| Figure | Value | Where |
|---|---|---|
| P0 violation escape rate, shield ON | **0.0** | canonical rail, 3 configs |
| P0 escape rate, shield OFF (control) | **0.626506** | proves the A/B can fail |
| Mean repair magnitude, shield ON | **4.07 m/s** (max 7.21) | new this period |
| Mean time to safe, shield ON | **0 episodes** — never unsafe | new this period |
| Mean time to safe, shield OFF | **4.1 s** | new this period |
| 10 m stand-off, shield OFF → ON | **7.07 m → 14.95 m** | 2.3 s → 0.0 s inside |
| Altitude recovery, before the fix | **9.99973 m** at 30 s, never enters | the defect |
| Altitude recovery, after | enters the band at **6.3 s** | the fix |
| Constraint classes | **6** (reference implementation: 2) | corridor + valid_time new |
| Scenario sweep | **12 scenarios, 11 pass, 1 known failure** | headless, ~1 s |
| Delivered runs rescored | **42**, every P0 figure reproduced | no re-flying |
| Test suite | **367 tests, 19 files** | all green |
| Camera-rail rate gate | **0 of 6** runs meet it | open item |
| OWL-ViT | **153M** params, 0.68 GB | not "100M" |
| Grounding DINO vs OWL-ViT on pedestrians | **0.327 vs 0.062** (5.2×) | but fails 4 Hz gate |

**Three sentences that carry the whole meeting.**
1. "The escape rate is zero, and this period it became a measurement rather than
   an inference."
2. "All five contractual KPIs are now computed — and the two we added found a
   defect that zero could never have shown."
3. "The camera evidence is still outside the contractual gate, and that is the
   next piece of work."

> **ID — tiga kalimat pembawa seluruh rapat.**
> 1. "Escape rate nol, dan periode ini ia menjadi **hasil pengukuran**, bukan
>    kesimpulan."
> 2. "Kelima KPI kontrak kini terhitung — dan dua yang kami tambahkan menemukan
>    cacat yang tak mungkin ditunjukkan oleh angka nol."
> 3. "Bukti berbasis kamera masih di luar gate kontrak, dan itu pekerjaan
>    berikutnya."
