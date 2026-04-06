Kompilieren:
$ cd src
$ cargo build --release
mit cargo 1.93.1 (083ac5135 2025-12-15) und rustc 1.93.1 (01f6ddf75 2026-02-11) auf Ubuntu 24.04.

Verwendung:
$ ./aufgabe3 <input_pfad> [output_pfad] [-D]
z.B.:
$ ./aufgabe3 lieferung00.txt lieferung00_out.txt

Die -D Flag gibt an, dass eine .dot GraphViz Datei für die Pfade ausgegeben werden soll, insofern das Problem lösbar ist.
Sie hat dann den Dateinamen der Input-Datei, nur mit der .dot-Extension.
Die Ausgabedatei hat standardmäßig den gleichen Dateinamen der Input-Datei, nur mit _out angehängt.
