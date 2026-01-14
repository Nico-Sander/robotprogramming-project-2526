# Dokumentation und Beantwortung der Aufgaben aus Task.md

## Optimierungsstrategie und Kollisionsbehandlung (Aufgabe 2a)

Die Glättung des initialen, stückweise linearen Pfades erfolgt durch die Implementierung der Klasse `OptimizeFlyby` und deren Hauptmethode `optimizePath`. Ziel des Verfahrens ist es, die abrupten Richtungsänderungen an den Wegpunkten durch $G^1$-stetige Übergänge zu ersetzen, wobei quadratische Bézier-Kurven zum Einsatz kommen. Die Strategie lässt sich in zwei wesentliche Phasen unterteilen: die Berechnung der virtuellen Kontrollpunkte (Relaxation) und die iterative Kollisionsbereinigung.

### 1. Berechnung der Pfadgeometrie durch Relaxation (Inverse Rounding)

Ein naives "Abrunden" der Ecken würde dazu führen, dass der Roboter den ursprünglichen Wegpunkt ($P_{org}$) "schneidet" und somit den Pfad verkürzt, was in engen Umgebungen zu Kollisionen an der Innenseite der Kurve führen kann. Um dies zu verhindern, implementiert der Algorithmus das Verfahren des **Inverse Rounding**.

Hierbei wird für jeden Knoten ein neuer, virtueller Kontrollpunkt $P_{2n}$ berechnet. Dieser Punkt wird so weit nach außen verschoben, dass der Scheitelpunkt der resultierenden Bézier-Parabel exakt den ursprünglichen Wegpunkt $P_{org}$ berührt. Die Berechnung folgt der Formel:

$$
P_{2n} = \frac{4 \cdot P_{org} - l_i \cdot P_{prev} - l_o \cdot P_{next}}{4 - l_i - l_o}
$$

Eine besondere Herausforderung, die die Methode `optimizePath` lösen muss, ist die gegenseitige Abhängigkeit der Punkte. Die Position des optimalen Kontrollpunkts $P_{2n}$ eines Knotens hängt von den Positionen seiner Nachbarn ($P_{prev}$ und $P_{next}$) ab. Da sich diese Nachbarn im selben Schritt ebenfalls verschieben, um ihre eigenen Kurven zu optimieren, entsteht ein gekoppeltes System.

Zur Lösung dieses Problems wendet der Algorithmus eine **Relaxations-Strategie** an. In einer inneren Schleife wird die Berechnung der Kontrollpunkte mehrfach (standardmäßig 15 Iterationen) über den gesamten Pfad hinweg wiederholt. Dies ermöglicht es den virtuellen Positionen, sich entlang der Kette zu propagieren und zu einem stabilen Zustand zu konvergieren, bevor die eigentliche Geometrie validiert wird.

### 2. Strategie zur Kollisionsbehandlung

Nachdem die ideale Geometrie basierend auf dem aktuellen Glättungsradius $r$ berechnet wurde, erfolgt die Validierung durch den `CollisionChecker`. Die Strategie zur Behandlung von Kollisionen ist **iterativ und degressiv**.

Initial wird jedem Knoten der konfigurierbare Radius `r_init` (maximal und standartmäßig $0.49$) zugewiesen. Der Algorithmus prüft anschließend in jedem Durchlauf der äußeren Optimierungsschleife vier kritische Bereiche auf Kollisionen:

1.  Das lineare Segment vom Startknoten zur ersten Kurve.
2.  Die linearen Verbindungssegmente zwischen den Kurvenenden ($E_{i}$) und den Kurvenanfängen ($S_{i+1}$).
3.  Das lineare Segment von der letzten Kurve zum Ziel.
4.  Die diskretisierten Bézier-Kurven selbst (überprüft mittels `curveInCollision`).

**Anpassungsstrategie bei Kollision:**
Sobald eine Kollision in einem Segment oder einer Kurve detektiert wird, wird der Pfad als ungültig markiert. Anstatt die Glättung für diesen Knoten vollständig zu verwerfen, wird der Radius $r$ für den betroffenen Knoten schrittweise um einen Wert `r_step` (konfigurierbar, standardmäßig $0.01$) reduziert.

* **Kollision in der Kurve:** Tritt eine Kollision direkt in der Kurve eines Knotens auf, wird nur dessen Radius verringert (`r_map[node] -= r_step`).
* **Kollision auf der Verbindungsstrecke:** Tritt eine Kollision auf dem geraden Segment zwischen dem Ende einer Kurve und dem Anfang der nächsten Kurve auf, werden die Radien **beider** angrenzenden Knoten reduziert.
    
    > **Hintergrund:** 
        Die Verbindungsstrecke weicht durch die Glättung vom ursprünglichen, kollisionsfreien Pfad ab. Durch das Verringern der Radien ziehen sich die Kurven näher an ihre ursprünglichen Eckpunkte zurück. Dadurch verschieben sich auch Start- und Endpunkt der Verbindungsstrecke wieder näher hin zum ursprünglichen Pfadsegment, welches garantiert hindernisfrei ist.

Da jede Änderung eines Radius die Geometrie und damit auch die Position der virtuellen Kontrollpunkte beeinflusst, erfordert eine Anpassung einen erneuten Durchlauf der Relaxationsphase. Dieser gesamte Zyklus wird iterativ so lange wiederholt (`max_iterations`), bis ein vollständig kollisionsfreier Pfad vorliegt oder die Radien ihren definierten Minimalwert (`r_min`) unterschreiten. Dies gewährleistet, dass der Algorithmus für jede Kurve konvergiert und dabei den maximal möglichen Radius beibehält, den die Hinderniskonstellation zulässt.

## Evaluation und Diskussion der Ergebnisse (Aufgabe 2c)

Zur Evaluierung des implementierten Verfahrens wurden die vier Benchmark-Szenarien herangezogen. Dabei wurden systematisch verschiedene Glättungsradien ($r$) getestet und deren Auswirkung auf die Pfadlänge, die Anzahl der Kollisionsberechnungen und die Rechenzeit analysiert.

### Analyse der Pfadlänge im Vergleich zum Originalpfad

Ein zentrales Ergebnis der Untersuchung ist das Verhältnis zwischen dem gewählten Radius $r$ und der resultierenden Gesamtlänge des Pfades.

* **Verlängerung durch Inverse Rounding:** Im Gegensatz zu klassischen Glättungsverfahren (z. B. Chaikin-Algorithmus oder einfaches Corner Cutting), welche den Pfad durch das "Abschneiden" von Ecken verkürzen, führt das hier implementierte **Inverse Rounding** zu einer **Verlängerung** der Wegstrecke gegenüber dem ursprünglichen, stückweise linearen Pfad.
* **Begründung:** Die Optimierungsstrategie erzwingt, dass die glättende Parabel den ursprünglichen Wegpunkt $P_{org}$ berührt (siehe Aufgabe 2a). Um dies bei steigendem Radius $r$ zu gewährleisten, muss der virtuelle Kontrollpunkt $P_{2n}$ geometrisch weiter nach außen geschoben werden. Der Roboter fährt also effektiv einen größeren Bogen *um* die theoretische Ecke herum, anstatt sie abzukürzen.
* **Linearer Zusammenhang:** Die Auswertungen (siehe "Radius Impact Analysis" Plot) zeigen eine näherungsweise lineare Korrelation zwischen dem Radius und der Pfadlänge. Dies entspricht der geometrischen Erwartung, da die Distanzverschiebung der Kontrollpunkte linear von den Parametern $l_i$ und $l_o$ abhängt, welche wiederum direkt proportional zu $r$ sind.

### Kollisionsberechnungen und Rechenzeit

Die Anzahl der Kollisionsberechnungen und die damit verbundene Rechenzeit korrelieren stark mit der Komplexität der Umgebung und dem gewählten Initialradius `r_init`.

* **Einfluss des Radius:** Wählt man einen sehr großen Initialradius in einer engen Umgebung (z. B. Benchmark 2 oder 4), steigt die Wahrscheinlichkeit, dass die weit nach außen geschobenen Kontrollpunkte oder die resultierenden Kurvensegmente in Hindernisse ragen.
* **Iterative Kosten:** Sobald eine Kollision detektiert wird, greift die in Aufgabe 2a beschriebene Reparaturstrategie (Reduktion von $r$). Dies erzwingt jedoch erneute Durchläufe der Optimierungsschleife, was die Anzahl der nötigen Kollisionschecks (`curveInCollision`, `lineInCollision`) und somit die Gesamtrechenzeit erhöht.
* **Vergleich:** In offenen Arealen (Teile von Benchmark 1) erfolgt die Berechnung nahezu instantan, da der initiale Radius sofort akzeptiert wird. In engen Passagen steigen die Berechnungskosten, bis der Radius so weit reduziert wurde, dass die Kurve kollisionsfrei ist.

### Zusammenfassende Diskussion

Das Verfahren erzeugt erfolgreich $G^1$-stetige Pfade, die für den Roboter ohne Stopps abfahrbar sind. Der Preis für das Beibehalten der ursprünglichen Wegpunkte (via points) ist eine Verlängerung der Strecke. Dies ist jedoch in vielen Anwendungsfällen (z. B. wenn der Wegpunkt eine Arbeitsstation oder ein Tor markiert, das exakt durchfahren werden muss) eine notwendige Eigenschaft. Das Verfahren stellt somit einen Kompromiss dar: Maximale Glätte (großes $r$) führt zu längeren Wegen und potenziell höheren Berechnungskosten in engen Umgebungen.

## Einfluss des Asymmetriefaktors $k$ (Aufgabe 2d)

Ergänzend zur Variation des Radius wurde die Konfiguration der Klasse `OptimizeFlyby` erweitert, um einen festen Asymmetriefaktor $k$ vorzugeben. Dieser Parameter modifiziert die Geometrie der Bézier-Parabeln, indem er das Verhältnis der Tangentenparameter am Kurveneingang und Kurvenausgang steuert.

### Implementierung und Wirkungsweise von $k$

In der Methode `optimizePath` definiert der Parameter $k$ das Verhältnis zwischen dem Parameter $l_i$ (für den Kurveneingang / "in") und $l_o$ (für den Kurvenausgang / "out"). Die Implementierung folgt der Beziehung:

$$
l_o = k \cdot l_i = k \cdot r
$$

Hierbei ist zu beachten, dass sich $k$ explizit als Skalierungsfaktor auf den **Kurvenausgang** bezieht. Ein $k > 1.0$ verlängert also den Tangentenabschnitt auf der Seite des ausgehenden Segments, während ein $k < 1.0$ ihn verkürzt.

### Unterschied zwischen dynamischer Symmetrie und $k=1.0$

Ein kritischer Aspekt bei der Evaluierung ist der Unterschied zwischen der Standardkonfiguration (`k=None`) und einer erzwungenen Symmetrie (`k=1.0`). Die Analyse des Codes zeigt, dass diese Modi fundamental unterschiedliche geometrische Eigenschaften erzeugen:

1.  **Dynamische / Geometrische Symmetrie (`k=None`):**
    Ist kein $k$ definiert, berechnet der Algorithmus die Tangentenpunkte so, dass der **absolute Abstand** (in Metern) vom Eckpunkt zum Startpunkt der Kurve ($S$) und zum Endpunkt der Kurve ($E$) identisch ist. Der Algorithmus orientiert sich dabei an der Länge des kürzeren Segments. Das Resultat ist ein geometrisch gleichschenkliges Dreieck an der Ecke.

2.  **Relative Symmetrie (`k=1.0`):**
    Wird $k=1.0$ gesetzt, bedeutet dies $l_o = l_i = r$. Da diese Parameter im Code jedoch als **Verhältnisfaktoren** (Prozentwerte der Segmentlänge) auf die Richtungsvektoren angewendet werden, sind die absoluten Abstände nur dann gleich, wenn auch die angrenzenden Pfadsegmente exakt gleich lang sind. Sind die Segmente unterschiedlich lang, liegen $S$ und $E$ zwar bei demselben prozentualen Anteil ihres jeweiligen Segments, haben aber unterschiedliche absolute Entfernungen zum Eckpunkt.

### Diskussion der Ergebnisse

Die durchgeführten Tests mit variablen $k$-Werten zeigen, dass die Wahl eines passenden Asymmetriefaktors signifikanten Einfluss auf die Qualität und Länge des Pfades hat.

* **Minimierung der Pfadlänge:** Die Ergebnisse zeigen typischerweise einen konvexen Verlauf der Pfadlänge über $k$. Es existiert oft ein optimales $k$ (häufig ungleich 1.0), bei dem die Pfadlänge minimal wird.
* **Geometrische Anpassung:** Durch die Wahl eines passenden $k$ kann der virtuelle Kontrollpunkt $P_{2n}$ so verschoben werden, dass der "Overshoot" (das notwendige Ausweichen nach außen beim Inverse Rounding) minimiert wird. Dies ist besonders effektiv in Szenarien mit stark unterschiedlichen Segmentlängen oder asymmetrischen Hindernissen. Ein flexibler $k$-Wert erlaubt es der Kurve, den verfügbaren Platz im längeren Segment stärker zu nutzen, um den Radius im kürzeren Segment nicht unnötig einschränken zu müssen.
* **Extreme Werte:** Sehr kleine oder sehr große Werte für $k$ führen dazu, dass die Kurve extrem auf eine Seite der Ecke verzerrt wird. Dies zwingt den virtuellen Kontrollpunkt weit nach außen, um den Kontakt zum ursprünglichen Wegpunkt zu halten, was die Pfadlänge drastisch erhöht.

### Bewertung der globalen Vorgabe und Performance-Aspekte

Abschließend lässt sich die Sinnhaftigkeit eines globalen $k$-Wertes sowie dessen Einfluss auf die Rechenzeit bewerten.

**Sinnhaftigkeit eines globalen $k$:**
Die Vorgabe eines einzigen, globalen Asymmetriefaktors für den gesamten Pfad erweist sich in der Praxis oft als suboptimal. Ein Pfad in komplexen Umgebungen besteht aus einer heterogenen Abfolge von Segmenten (kurz-lang, lang-kurz, lang-lang).
* Ein globales $k \neq 1$ erzwingt eine einheitliche Verzerrung aller Kurven (z. B. immer "kurzer Eingang, langer Ausgang").
* Während dies einer spezifischen Ecke zugutekommen kann (z. B. Übergang von einem engen Korridor in einen freien Raum), verschlechtert es zwangsläufig Ecken mit umgekehrter Topologie (Übergang von freiem Raum in einen Korridor).
* Daher ist ein globales Optimum für $k$ meist nur ein Kompromiss, der den "durchschnittlichen Fehler" minimiert, aber selten jede Ecke ideal glättet.

**Rechenleistung und Zeitverhalten:**
Die Implementierung im Code zeigt zwei Aspekte bezüglich der Performance:
1.  **Berechnungsaufwand pro Schritt:** Die mathematische Anwendung von $k$ innerhalb der Funktion `get_tangent_points` ist trivial und erzeugt keinen messbaren Mehraufwand gegenüber der symmetrischen Berechnung ($O(1)$).
2.  **Einfluss auf die Konvergenz:** Ein ungünstig gewählter globaler $k$-Wert kann die Rechenzeit indirekt signifikant erhöhen. Zwingt das globale $k$ die Kurvengeometrie in Hindernisse (Kollision), greift der in Aufgabe 2a beschriebene Reparaturmechanismus. Dies führt dazu, dass die äußere Schleife (`max_iterations`) häufiger durchlaufen werden muss, um den Radius $r$ schrittweise zu reduzieren, bis die durch $k$ verzerrte Kurve kollisionsfrei ist.