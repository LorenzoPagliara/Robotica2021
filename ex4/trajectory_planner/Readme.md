# trajectory_planner

Questo pacchetto fornisce un planning node che carica un percorso da un file `.traj` presente nella cartella `/data` e genera una traiettoria nello spazio dei giunti. Consente inoltre all'utente di verificare che i limiti di giunto siano rispettati pubblicando le traiettorie su `rqt_multiplot`.

## Lancio del pacchetto

L'esecuzione del pianificatore richiede che il parametro `robot_description` sia caricato sul parameter server. Quindi si esegue in un terminale 

```bash
roslaunch fanuc_moveit_config demo.launch
```

Il planner viene lanciato in un altro teminale eseguendo

```bash
roslaunch trajectory_planner fanuc_planner.launch
```