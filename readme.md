# Pemetaan 3D dengan Drone di CoppeliaSim

Proyek ini mendemonstrasikan pemetaan 3D otomatis menggunakan Quadcopter yang dilengkapi dengan *Perspective Vision Sensor* di CoppeliaSim. Drone ini terbang secara otonom di atas area yang telah ditentukan untuk memindai objek geometris (kotak dan silinder) dan menghasilkan peta *point cloud* 2D dan 3D. Data tersebut kemudian diproses lebih lanjut (*post-processing*) untuk melakukan *downsampling voxel* dan pewarnaan berdasarkan ketinggian objek.

## 🚀 Fitur & Kriteria Penilaian yang Dipenuhi

Repositori ini dibuat untuk memenuhi tugas hierarkis berikut:

* **Grade C:** Kontrol drone otonom dan visualisasi jalur penerbangan (`--mode path`).
* **Grade BC:** Pemindaian 2D menggunakan sensor kedalaman (*depth sensor*) satu baris yang berputar (`--mode 2d`).
* **Grade B:** Pemindaian 3D menggunakan kamera kedalaman yang dimiringkan dan pola lintasan *lawn-mower* (`--mode 3d`).
* **Grade AB:** Optimasi *point cloud* dengan mengurangi titik menjadi *voxel* (*voxel downsampling*).
* **Grade A:** *Post-processing* pewarnaan peta berdasarkan ketinggian (Sumbu Z).

## 📋 Prasyarat

* **Simulator:** [CoppeliaSim](https://www.coppeliarobotics.com/) (Versi >= 4.5 direkomendasikan untuk dukungan ZMQ Remote API).
* **Python 3.x**
* **Paket Python:**
  ```bash
  pip install coppeliasim-zmqremoteapi-client numpy matplotlib
  pip install open3d  # Opsional, untuk tampilan 3D tingkat lanjut
  ```

## 📂 Struktur Repositori

```text
.
├── ETS Mapping.ttt       # File scene utama CoppeliaSim
├── drone_mapping.py      # Skrip Python untuk mengontrol drone dan mengambil data
├── postprocess.py        # Skrip Python untuk menyaring, membuat voxel, dan memplot peta
├── README.md             # Dokumentasi proyek
└── output/               # Folder otomatis untuk point cloud dan plot
    ├── path_3d.csv
    ├── scan3d.npy
    ├── map3d.png
    └── ...
```

## 🛠️ Petunjuk Penggunaan

### 1. Menyiapkan Simulasi & Sensor

1. Buka **CoppeliaSim**.
2. Muat file *scene*: `File` > `Open scene...` > Pilih `ETS Mapping.ttt`.
3. **Pengaturan Sensor Manual (Wajib):** Anda harus membuat dan mengatur sensor di dalam *scene* agar skrip dapat berjalan.
   * Pada panel *Scene Hierarchy* (kiri), klik kanan > **Add** > **Vision sensor** > **Perspective type**.
   * Ubah nama sensor tersebut menjadi `mapping_sensor`.
   * Tarik `mapping_sensor` ke dalam objek `Quadcopter` (jadikan sebagai *child*).
   * Klik dua kali pada ikon `mapping_sensor` untuk membuka propertinya dan atur nilai berikut:
     * **Resolution X / Y**: `128` / `128`
     * **Near clipping plane**: `0.50`
     * **Far clipping plane**: `12.00`
     * **Perspective angle**: `60.00`
   * **SANGAT PENTING:** Centang kotak **Explicit handling** pada menu properti sensor (ini memungkinkan Python untuk mengontrol kapan kamera mengambil gambar).

### 2. Menjalankan Pengambilan Data (Penerbangan & Pemindaian)

Buka terminal Anda di direktori proyek dan jalankan skrip pemetaan. **Jangan hentikan simulasi secara manual**; biarkan skrip menyelesaikan lintasannya dan mencetak `[done]` di terminal untuk memastikan data tersimpan utuh.

* **Untuk Pemetaan 3D Penuh (Grade B):**
  ```bash
  python drone_mapping.py --mode 3d
  ```

* **Untuk Pemetaan 2D (Grade BC):**
  ```bash
  python drone_mapping.py --mode 2d
  ```

* **Hanya untuk Visualisasi Jalur (Grade C):**
  ```bash
  python drone_mapping.py --mode path
  ```

*(Data penerbangan akan disimpan sebagai file `.npy` dan `.csv` di dalam direktori `output/`).*

### 3. Post-Processing & Visualisasi

Setelah penerbangan selesai, proses data mentah yang dihasilkan untuk membuat peta akhir.

* **Voxel Downsampling & Pewarnaan Ketinggian (Grade AB & A):**
  ```bash
  python postprocess.py --mode 3d --voxel 0.15
  ```
  Perintah ini membersihkan data, menyusutkan *point cloud* ke dalam *voxel* berukuran 0.15m, mewarnainya berdasarkan ketinggian, dan menampilkan plot 3D beserta jalur terbang drone.

* **Melihat Data Pindai 2D:**
  ```bash
  python postprocess.py --mode 2d
  ```

* **Tampilan Tingkat Lanjut (Open3D):**
  Jika Anda telah menginstal `open3d`, Anda dapat menggunakan flag `--o3d` untuk melihat *point cloud* dalam lingkungan 3D yang lebih interaktif:
  ```bash
  python postprocess.py --mode 3d --voxel 0.15 --o3d
  ```

## ⚠️ Pemecahan Masalah (Troubleshooting)

* **Error "object not tagged for explicit handling":** Anda lupa mencentang opsi **Explicit handling** pada properti `mapping_sensor` di CoppeliaSim. Centang opsi tersebut lalu jalankan ulang skrip.
* **Data Kosong/Titik Hilang:** Pastikan parameter fungsi get depth di Python disetel ke nilai normalisasi, yakni `sim.getVisionSensorDepth(sensor, 0)` (menggunakan argumen `0`).
* **Peta Terbalik (Mirrored):** Jika plot 3D terlihat terbalik posisinya dibandingkan dengan *scene* asli di CoppeliaSim, buka file `drone_mapping.py`, lalu ubah variabel `FLIP_U = True` atau `FLIP_V = True` di bagian atas file, kemudian lakukan pemindaian ulang.
