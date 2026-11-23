# Suunnittelupäätökset ja kurssin konseptit

Tämä dokumentti kokoaa keskeiset tekniset ratkaisut, niiden perustelut kurssin konseptien kautta sekä todentamisen. Se on pohja esitysdioille.

## 1. Lähtötilanne ja autonomian perustelu

- Lastit: kivihiili (IMSBC ryhmä B, kemiallinen vaara), kalkkikivi ja rautapelletit (ryhmä C).
- Nykytila: ruumissa ei ole antureita, tuuletuspäätökset tehdään käsin, miehistö mittaa kannettavalla mittarilla kerran päivässä.
- Autonomian lisäarvo: jatkuva valvonta, paikallinen tieto (missä ruuman osassa kuumenee), ristiriitaisten tavoitteiden punninta ja se, ettei miehistön tarvitse mennä kaasuvaaralliselle luukulle mittaamaan.

## 2. Havainnointi (Sensing and perception, L2–L3, tehtävä 2)

| Ratkaisu | Perustelu |
|---|---|
| Kaksi anturityyppiä: kiinteä poistoanturi (jatkuva) ja vaunun sondi (pistemäinen, vyöhykkeittäin) | Poistoanturi näkee suljetussa ruumassa vain oman vyöhykkeensä ilman, ja tuuletettuna kaikkien vyöhykkeiden sekoituksen. Kuumenemiskohta voi olla kaukana. |
| CO:n tuottonopeus massataseesta: g = dCO/dt + k·CO − m·(naapurit − 2·CO) | Raaka CO-pitoisuus riippuu tuuletuksesta (toimilaitteen tila vaikuttaa havainnon tulkintaan). Tuottonopeus ei riipu siitä, joten tunnistus toimii myös tuulettimen käydessä. Mallipohjainen tila-arvio (world model, L3). |
| Sekoittumistermi naapurivyöhykkeiden näytteistä | Ilman sitä kuuma vyöhyke aliarvioitui ja naapuri yliarvioitui: paikannus osoitti ensimmäisessä versiossa väärään vyöhykkeeseen. |
| Vahvistus kahdesta peräkkäisestä arviosta | Yksittäinen kohinainen arvio ei laukaise hälytystä (kompromissi nopeuden ja väärien hälytysten välillä). |
| Metaanille nopeampi suodatin (τ = 0,2 h) kuin CO:lle (1 h) | 1 h suodatin viivästi PI-säätöä niin, että metaani ehti 23 %:iin LEL käynnistyksessä. Sama suodatuksen ja viiveen kompromissi kuin tehtävässä 2. |
| Anturivian tunnistus: poistoanturin CO vs. vaunun näytteistä laskettu odotusarvo | Redundanssi kahden anturityypin välillä (sensor fusion). Ryömivä poistoanturi olisi yksin antanut väärän itsekuumenemishälytyksen. |
| Näyte hyväksytään vain, kun sondi on asettunut aukkoon | Mittauksen kelpoisuus riippuu toimilaitteen tilasta: sondin anturilla on aikavakio, ja liikkeessä se mittaa kannen ilmaa. |

## 3. Päätöksenteko (Reasoning and decision-making, L4–L5, tehtävä 3)

Käyttäytymispuu (py_trees), prioriteettijärjestyksessä:

```
Root (Sequence)
  Plan sampling            vaunun prioriteettivyöhyke
  Check sensors            anturivika -> ADVISORY
  Decide (Selector)
    Methane emergency      CH4 >= 20 % LEL (pito 15:een)  -> PURGE
    Methane control        CH4 > 5 % LEL (pito 3:een)     -> CONTROL_CH4 (PI)
    Self-heating           vahvistettu kuumenemiskohta    -> CLOSED (happi pois)
    Monitor                                                 -> CLOSED
```

| Ratkaisu | Perustelu |
|---|---|
| Metaani ennen itsekuumenemista | Räjähdysvaara on akuutimpi; IMSBC-tulkinta. Reaktiivinen kerros ensin, kuten tehtävän 3 Selectorissa. |
| Hystereesi kynnyksissä | Estää tilan värähtelyn kynnyksen ympärillä. |
| Ristiriidassa asetusarvo nostetaan 10 → 15 % LEL | Monitavoitteinen punninta: vähemmän ilmaa = hitaampi kuumeneminen, mutta pienempi marginaali 20 %:n rajaan. Testattu 10/12/15 % (alla). |
| Inertointia ei automatisoida | Autonomian taso: järjestelmä tunnistaa ja hälyttää (CREW_ACTION), ihminen päättää peruuttamattomasta toimenpiteestä (human-in-the-loop). |
| Näytteenoton suunnittelu: epäilyttävä vyöhyke joka toisella käynnillä | Havainnoinnin resurssin (vaunun aika) suunnittelu epävarmuuden mukaan (L5): tieto tarkentuu siellä, missä sitä tarvitaan. |
| Ei 24 h alkutuuletusta, jos metaania ei mitata | Jatkuva mittaus korvaa varmuuden vuoksi tehtävän tuuletuksen. Itsekuumenevalla hiilellä tämä piti kuumenemiskohdan 38 °C:ssa (käsimenettely 52 °C). |

Metaanin asetusarvo ristiriitatilanteessa (coal_both, kaksi satunnaissiementä):

| Asetusarvo | Kuumenemiskohta ≥ 60 °C | CH₄ max | Keskim. ilmavirta |
|---|---|---|---|
| 10 % LEL | 32,3 h | 14,4 % | 0,25 |
| 12 % LEL | 33,3 h | 14,4 % | 0,21 |
| 15 % LEL | 34,8 h | 15,2 % | 0,17 |
| Käsimenettely | 34,5 h | 59 % | 0,53 |

Asetusarvolla 10 % jatkuva vähäinen tuuletus toi happea niin, että lasti kuumeni nopeammin kuin käsimenettelyssä, jossa on täysin suljettuja jaksoja. Asetusarvolla 15 % kuumeneminen on käsimenettelyn tasolla, mutta metaani ei ylitä 20 %:n rajaa.

## 4. Ohjaus (Act and control, L6–L7, tehtävä 4)

### Tuuletus: PI-säätö
- Säädettävä suure CH₄ (% LEL), ohjaussuure tuulettimen nopeus (ilmavirta 0–1).
- Tavoite: metaani asetusarvossa pienimmällä mahdollisella ilmavirralla, jotta happea tulee mahdollisimman vähän.
- Anti-windup (ehdollinen integrointi), lähdön rajat, minimivirtaus 5 % (pelti pysyy auki, ei edestakaista kytkentää), nykäyksetön siirtyminen PURGE → CONTROL.
- Lukitus mitatun tilan perusteella: tuuletin käynnistyy vasta kun pelti on auki, pelti sulkeutuu vasta kun tuuletin on pysähtynyt.

Viritys (coal_methane, 120 h):

Ensimmäinen viritys (CH₄-suodatin 1 h, ei minimivirtausta):

| Kp | Ki | CH₄ max | CH₄ keskihajonta 24 h jälkeen | Tuuletuksen päälle/pois-kytkennät |
|---|---|---|---|---|
| 0,05 | 0,10 | 23,3 | 0,53 | 7 |
| 0,05 | 0,20 | 23,3 | 3,01 | 40 |
| 0,10 | 0,05 | 23,2 | 0,07 | 1 |

Liian suuri Ki teki säädöstä värähtelevän. Käynnistyksen 23 %:n huippu ei riippunut vahvistuksista vaan suodattimen viiveestä, joten CH₄-suodatin nopeutettiin (τ = 0,2 h) ja lisättiin minimivirtaus. Lopullinen viritys:

| Kp | Ki | Kd | CH₄ max | CH₄ keskihajonta 24 h jälkeen | Ilmavirran keskim. muutos / 15 min |
|---|---|---|---|---|---|
| 0,10 | 0,05 | 0 | 14,4 | 0,07 | 0,012 |
| 0,10 | 0,05 | 0,1 | 13,3 | 0,17 | 0,077 |
| 0,10 | 0,05 | 0,3 | 13,0 | 0,32 | 0,103 |

D-termi pienensi huippua hieman, mutta vahvisti anturikohinaa: ilmavirta heilui 6–9 kertaa enemmän ja metaanin vaihtelu kasvoi. Valinta Kp = 0,1, Ki = 0,05, Kd = 0. Ensimmäisessä versiossa (päälle/pois-ohjaus) tuuletin kytkeytyi tiheästi edestakaisin.

### Vaunu: tilakone + PID-asentosäätö
- Tilat SELECT → MOVE → LOWER → SETTLE → SAMPLE → RAISE, aikakatkaisu → FAULT (sondi ylös, odotus, uusi yritys).
- Saapumisehto: asentovirhe < 1 cm **ja** nopeus < 2 cm/s vähintään 0,3 s. Tehtävän 4 pick and place -kokemus: pelkkä kynnys hyväksyy "melkein perillä, vielä liikkeessä", ja ilman aikakatkaisua tilakone voi jäädä jumiin.
- Asettumisaika SETTLE 1,5 s: sondin anturin aikavakio 0,5 s.

PID-viritys (askel 2,85 m, kiihtyvyysraja 1 m/s², max 0,8 m/s):

| Kp | Kd | Asettumisaika | Ylitys |
|---|---|---|---|
| 1,5 | 0 | 5,9 s | 0 cm |
| 3,0 | 0 | 5,3 s | 6,3 cm (yli aukon 5 cm toleranssin) |
| 3,0 | 0,3 | 4,7 s | 0 cm |
| 5,0 | 0 | 5,4 s | 17,1 cm |

Vaunussa D-termi on hyödyllinen, koska kiihtyvyysraja toimii kuin hitaus. Tuuletuksessa D-termi vain vahvisti kohinaa. Sama säädinrakenne, eri prosessi: termin hyöty riippuu prosessin dynamiikasta.

## 5. Todentaminen ja testaus

1. **Yksikkötestit** (29 kpl, `pytest`): prosessimallin fysiikka, sondin mittaus vain aukossa, CO-tuoton arvio eri tuuletuksilla, sekoittumiskorjaus, anturivika, päätöspuun jokainen haara ja hystereesit, PID:n anti-windup, tuuletuksen lukitukset, vaunun kierto, prioriteetti ja aikakatkaisu.
2. **Suljetun silmukan simulaatio** samoilla logiikkaluokilla ja Webotsin toimilaitedynamiikalla: viisi skenaariota, vertailu IMSBC-käsimenettelyyn, viritys. Toistettavuus tarkistettu viidellä satunnaissiemenellä: ei vääriä hälytyksiä, kuumenemiskohta aina oikein.
3. **Webots-integraatio**: kaikki solmut ja driver oikeaa maailmaa vasten, 5 × 120 h. Tulokset vastaavat simulaatiota (itsekuumenemisen tunnistus Webotsissa 10,3 h, simulaatiossa 16,9 h; ero johtuu siitä, milloin vaunu osuu kuumaan vyöhykkeeseen).
4. **Todellinen tila vs. arvio**: `plant_sim` julkaisee todellisen tilan (`/hold/truth`), jota verrataan havainnoinnin arvioon kuvaajissa.

## 6. Tulokset

Taulukko README:ssä. Keskeisimmät:
- Metaani ei ylitä 20 % LEL yhdessäkään skenaariossa (käsimenettelyssä 28–42 h yli).
- Itsekuumeneminen tunnistetaan ja paikannetaan 10–17 h kohdalla, käsimenettelyssä 32 h.
- Ristiriitatilanteessa miehistö saa 22 h enemmän aikaa päättää inertoinnista. Kuumeneminen itsessään etenee yhtä nopeasti kuin käsimenettelyssä: tuuletus ei voi ratkaista ristiriitaa, vaan se vaatii ihmisen päätöksen.

## 7. Rajoitukset ja jatkokehitys

- Prosessimalli on laadullinen; parametrit (ilmanvaihto, sekoittuminen) oletetaan tunnetuiksi. Jatkossa ne voisi tunnistaa mittauksista.
- Vaunu kulkee yhtä kiskoa yhden ruuman yli; useampi ruuma vaatisi reitinsuunnittelua.
- Hälytys näkyy vain merkkivaloina ja lokissa; todellinen järjestelmä tarvitsisi käyttöliittymän ja kuittauksen.
