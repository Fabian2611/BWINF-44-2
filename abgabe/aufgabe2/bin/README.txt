Kompilieren:
$ cd src
$ cargo build --release
mit cargo 1.94.1 (29ea6fb6a 2026-03-24) und rustc 1.94.1 (e408947bf 2026-03-25) auf Ubuntu 24.04.

Verwendung:
$ ./aufgabe3 <input_pfad> [output_pfad]
z.B.:
$ ./aufgabe3 roboter01.txt roboter01_out.txt

Die Ausgabedatei hat standardmäßig den gleichen Dateinamen der Input-Datei, nur mit _out angehängt.
