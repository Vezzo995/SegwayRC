Version 1.0

1) Added the Kalman filter to increase the speed:

Dati che mi servono (e come li ricavi in pratica)

m: 1,600 kg
massa totale del “corpo” sopra alle ruote (kg) – escludi la massa delle ruote se misuri separatamente I_w.

l: 0,08 m
distanza del baricentro dall’asse ruote (m) (verticale quando in equilibrio).

I_b: 0,0106112 kg·m²
momento d’inerzia del corpo attorno all’asse ruote (kg·m²). Se non lo sai, possiamo stimarlo:
– misura m e l, usa I_b ≈ m·l²·k con k tra 0.7 e 1.2 (dipende da come è distribuita la massa).

r: 0,0525 m
raggio ruota (m) = (diametro/2).

I_w: 0,0000826875 kg·m²
inerzia di ciascuna ruota+mozzo (kg·m²). Se non nota, stimiamo da massa ruota m_w=30gr e r=52,5mm (I_w ≈ ½ m_w r²).

N: 1
rapporto di riduzione (se c’è). Se direct drive, N=1.

K_t: costante di coppia motore (N·m/A).

K_e: costante di forza controelettromotrice (V·s/rad). Spesso K_t ≈ K_e in unità coerenti.

R_m: resistenza elettrica vista dal motore (Ω), includi driver se noto.

V_bat: tensione batteria (V).

bφ: attrito viscoso equivalente sul corpo (N·m·s/rad), se ignoto possiamo iniziare da 0.

bw: attrito viscoso vista ruota (N·m·s/rad), inizialmente 0.

ticks_per_rev: impulsi encoder per giro ruota (dopo riduzione).

dt: periodo di campionamento (s), tipico 0.005–0.01 (200–100 Hz).

σgyro, σacc, σenc: deviazioni standard rumore dei sensori (rad/s, rad, m o rad). Le stimiamo dai tuoi log (vedi sotto).
