# coye_tesla

Simulation Python d'un résonateur Tesla monté au centre d'une plaque ronde avec:
- calcul du champ magnétique (centre et bords à 0°, 90°, 180°, 270°),
- 4 bâtons inclinés de 15° vers le haut,
- fil de cuivre en carré reliant les extrémités,
- 4 feuilles d'aluminium pliées autour des fils,
- estimation des forces électromagnétiques et du déplacement 3D sous 12 V.

## Exécution

```bash
python3 tesla_resonator_sim.py
```

Le script affiche les résultats physiques dans le terminal et exporte une géométrie 3D (`tesla_resonator_geometry.obj`).
