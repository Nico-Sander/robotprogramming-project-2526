# Projektaufgabe - Optimierung eines geglätteten Ergebnispfades

## Beschreibung

In der Vorlesung haben wir festgestellt, dass das Glätten eines Lösungspfades, z.B. durch Bechthold-Glavina, unnötige Umwege entfernt und die Pfade gut um Hindernisse herumliegen. Der Roboter muss jedoch an jedem Punkt anhalten. Es gibt die Möglichkeit, Überschleifen (Rounding / via Points) einzusetzen, um hier die Bahn noch weiter zu optimieren.
Implementieren Sie die Optimierung auf Basis der Informationen aus `IP-10-1-Smoothing-slides.pdf`: `Model of rounding: first try` (slide 93) und `Inverse rounding: first model`

### Aufgabe 1

Da Sie keinen automatiesierten Glätter haben, der zuvor geglättete Bahnen erstellen kann, erstellen Sie zunächst 4 unterschiedliche Benchmarkszenarien für 2-DoF-Punktroboter "per Hand", d.h. Sie erzeugen manuell einen geglätteten kollisionsfreien Pfad, d.h.:

#### a)
CollisionChecker mit der Szene alngegen.

#### b)
Planer auswählen und mit CollisionChecker erzeugen.

#### c)
Per Hand in den Graphen des Planers die Punkte / Knoten eintragen, die einem zuvor geglätteten kollisionsfreien Lösungspfad entsprechen.

#### d)
Lösungspfad = Liste mit Namen der Punkte dieses Pfades.

**Anmerkung**
Achten Sie darauf, dass im Pfad mehr als 5 Punkte enthalten sind und dass die Pfadsegmente nich immer die gleiche Länge haben.

### Aufgabe 2

Erzeugen Sie dann eine Klasse *OptimizeFlyby* mit jeweils einer Member-Funktion *optimizePath* mit der folgenden Signatur:

`optimizePath(Pfad, Planer, Config)`

**Pfad:**
Kollisionsfreier geglätteter Lösungspfad, der in diesem Fall per Hand erzeugt wurde.

**Planer:**
Eine Instanz eines Planers. Über diese Instanz haben Sie über die Membervariable *graph* der Planer-Klasse Zugriff auf die im Lösungspfad referenzierten Knoten und deren Positionen, sowie den Zugriff auf den *self._collisionChecker*.

**Config:**
Dictionary mit Parametern / Konfiguration für die Glättung.

**Rückgabe:**
Der optimierte Pfad (Namen der Knoten + Überschleifwert an jedem Punkt).

**Anmerkung:**
Punkte / Knoten, die während der Optimierung erzeugt werden und für den Lösungspfad oder auch die Visualisierung benötigt werden, speichern sie im Graphen des Planers ab.

#### a)
Gehen Sie davon aus, dass `r` (siehe Slides) zunächst via Config fest zwischen `0.02 - 0.49` vorgegeben werden kann. Versuchen Sie zunächst auf der Basis dieses Wertes den Pfad zu optimieren, indem Sie die virtuellen Punkte erzeugen und die entsprechenden Pfadsegmente und Parabeln auf Kollision überprüfen. Bei Kollision passen sie `r` für den entsprechenden Punkt an, indem sie den Wert nach einer von Ihnen gewählten Strategie verkleinern. Erläutern Sie die Strategie.

#### b)
Erzeugen Sie eine Visualisierungsfunktion, die die verschiedenen Segmente und Parabeln farblich hervorhebt.

#### c)
Testen, Visualisieren und Evaluieren Sie das Verfahren anhand der 4 Benchmarkszenarien. Stellen Sie grafisch die Anzahl der Kollisionsberechnungen, Berechnunszeit und die Länge des Pfades (hier müssen Sie noch eine Berechnung für die Parabeln erarbeiten) dar und vergleichen Sie die Länge zum uprsprünglichen Pfad. Diskutieren Sie die Ergebnisse.

#### d)
Erweitern Sie noch die Möglichkeit `k` in $z_{neu}(k) = P_2$ (siehe Slide 101) in der Config vorgeben zu können. Und führen Sie die Tests aus *c)* nochmals für unterschiedliche Werte von `k` durch. Diskutieren Sie die Ergebnisse.

#### e)
Ermitteln Sie das optimale `k` nachdem ihr Algorithmus für ein gegebenes `r` durchgelaufen ist. Optimal ist, wenn dann die Länge der Strecke möglichst klein wird. Diskutieren Sie die Ergebnisse.

### Aufgabe 3
Erläutern und diskutieren Sie im Endbericht. Wie kann der Lösungspfad mit einem Industrieroboter abgefahren werden? Welche Bewegungsbefehle könnten Sie einsetzen?
