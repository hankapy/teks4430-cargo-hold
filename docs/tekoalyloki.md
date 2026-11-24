# Tekoälyn käyttö

Työkalu: Claude (Opus 5.5, Claude-sovellus).

| Mihin käytettiin | Oma osuus / tarkistus | Havainnot |
|---|---|---|
| Aiheen valinta (varustamon ideat), IMSBC-menettelyn selvitys, alustava arkkitehtuuri | Aiheen valinta ja rajaukset (ei antureita, lastit, käsin tehtävät päätökset) | |
| Tehtävänannon kattavuuden tarkistus, prosessimalli ja IMSBC-vertailu erillisenä skriptinä | | Ensimmäinen versio tunnisti itsekuumenemisen ristiriitaskenaariossa myöhemmin kuin käsimenettely, koska tuuletus laimensi CO-lukemat. Korjattiin CO:n tuottonopeuden arviolla (massatase). |
| ROS 2 -paketit, viestityypit, plant_sim, Webots-maailma ja driver | | Claude testasi maailman ja driverin Webotsilla omassa ympäristössään. Asentoinkrementeillä ohjattu tuuletin ylitti tavoitenopeuden, joten ohjaus vaihdettiin nopeusohjaukseen. Ensimmäisessä maailmatiedostossa oli virheellisiä ylätason DEF-määrittelyjä ja virheellinen taustatekstuuri. |
| Kysymys: täyttääkö työ tehtävänannon, kun liikkuvia osia ei ole? | Päätös toteuttaa sekä PI-säätö että näytteenottovaunu | Claude arvioi, ettei liikkuvaa robottia vaadita, mutta ohjauskerros oli päälle/pois-tilakoneena ohut verrattuna muihin kerroksiin. |
| Vyöhykkeinen malli, havainnointi, käyttäytymispuu, PI-säätö, vaunun tilakone ja PID, kaikki solmut, maailman kisko ja vaunu | | Logiikka erotettiin ROS-riippumattomiin moduuleihin, jotta sen voi testata ilman ROSia. Claude ei saanut ROS 2:ta asennettua omaan ympäristöönsä; solmut testattiin Webotsia vasten kevyellä rclpy-korvikkeella, jossa viestien tyyppitarkistukset olivat yhtä tiukat kuin ROS 2:ssa. Oikea colcon build ja launch testattava omalla koneella. |
|Viritys ja korjaukset suljetun silmukan simulaatiolla | | Löydetyt virheet: (1) PI:n anti-windup-ehto toimi väärin negatiivisilla vahvistuksilla, (2) vyöhykkeiden sekoittuminen aluksi liian voimakasta, kuumenemiskohtaa ei voinut paikantaa, (3) massatase ilman sekoittumistermiä paikansi kuumenemiskohdan väärään vyöhykkeeseen, (4) 1 h CH₄-suodatin viivästi PI-säätöä, (5) kisko oli kameran näkymättömällä puolella. Havainto: ristiriitatilanteessa autonominen järjestelmä kuumensi lastia aluksi nopeammin kuin käsimenettely, koska jatkuva vähäinen tuuletus toi happea; korjattiin nostamalla metaanin asetusarvoa ristiriidassa. Ensimmäinen viritystaulukko käytti harhaanjohtavaa kytkentämittaria (kynnys osui minimivirtaukseen), korjattiin. |
| VS Coden tekoälyavustajaa käytettiin muutaman koodin kohdan auki selittämiseen. |
