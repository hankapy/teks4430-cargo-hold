# Webots-ajojen tulokset (oma kone, 23.11.2026)

Ympäristö: Windows, Webots R2025a natiivisti, ROS 2 Jazzy Docker-kontissa. Aikaskaala 600, kesto 120 h. Tunnusluvut: `analysis/plot_run.py`. Kuvaajat: `docs/results/own_*.png`.

| Skenaario | Loki | Itsekuumeneminen vahvistettu | Kuumenemiskohta | Miehistöhälytys | Anturivika | CH₄ max (% LEL) | Tunnit CH₄ > 20 % | Max T (°C) | T ≥ 60 °C | Keskim. ilmavirta |
|---|---|---|---|---|---|---|---|---|---|---|
| coal_methane | coal_methane_20261123_125114 | – | – | – | – | 16,8 | 0 | 15,0 | – | 0,2 |
| coal_selfheat | coal_selfheat_20261123_130735 | 15,6 h | vyöhyke 1 (oikein) | 61,0 h | – | 0,0 | 0 | 38,1 | – | 0,0 |
| coal_both | coal_both_20261123_130851 | 15,6 h | vyöhyke 1 (oikein) | 15,6 h | – | 16,7 | 0 | 150 (malli leikkaa) | 34,6 h | 0,2 |
| limestone | limestone_20261123_131007 | – | – | – | – | 0,0 | 0 | 15,0 | – | 0,0 |
| limestone, CO-ryömintä 1 ppm/h | limestone_20261123_131126 | – | – | – | 49,7 h | 0,0 | 0 | 15,0 | – | 0,0 |

Savutesti `coal_both_20261123_124828` (20 h): itsekuumeneminen vahvistettu 15,4 h, vyöhyke 1.

## Vertailu simulaatioon ja käsimenettelyyn

| Mittari | Käsimenettely (sim.) | Simulaatio | Oma Webots-ajo |
|---|---|---|---|
| Metaani: tunnit yli 20 % LEL | 41,5 h | 0 h | 0 h |
| Itsekuumeneminen: tunnistus | 32 h | 16,9 h | 15,6 h |
| Itsekuumeneminen: max T | 52 °C | 38 °C | 38,1 °C |
| Molemmat: miehistöhälytys | 32 h | 10,0 h | 15,6 h |
| Molemmat: T ≥ 60 °C | 34,5 h | 35,1 h | 34,6 h |
| Kalkkikivi: väärät hälytykset | – | 0 | 0 |
| Ryömintä: anturivika tunnistettu | – | 35,7 h | 49,7 h |

## Havainnot

- Metaanin huippu (16,8 %) on korkeampi kuin simulaatiossa (14,4 %). Ero syntyy käynnistyksessä: Webotsissa pelti avautuu noin 2 sekunnissa (noin 20 prosessiminuuttia) ennen kuin lukitus sallii tuulettimen käynnistyä.
- Tunnistusaika (15,4–15,6 h) oli omilla ajoilla lähes vakio, mutta erosi simulaatiosta (10–17 h). Tunnistus vaatii kaksi peräkkäistä arviota kuumasta vyöhykkeestä, joten näytteenottoväli määrää alarajan.
- Ryömintä tunnistettiin myöhemmin kuin simulaatiossa, mutta väärää itsekuumenemishälytystä ei tullut, koska päätös perustuu vaunun näytteisiin.
- Ajon lopussa löytyi puute: solmut jatkoivat ajoa skenaarion päätyttyä. Korjattu niin, että plant_sim sammuttaa koko launchin.
