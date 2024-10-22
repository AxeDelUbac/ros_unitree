rendre un fichier python exacutable
```bash
chmod 755 my_first_node.py
```

afficher les noeud Ros en cours
```bash
rosnode list
```

afficher les liens entre les noeud ROS
```bash
rqt_graph
```

Lister les topc
```bash
rostopic list
```

Avoir des infos sur un topic
```bash
rostopic info /nom_topic
```

Voir les composantes du topic
```bash
rosmsg show typedemessage
```

Visualiser les donnée d'un topic en temps réel
```bash
rostopic echo /nomdutopic
```

Connaitre le type utiliser par un topic
```bash
ros topic info
```

Lancer la turtlesim
```bash
rosrun turtlesim turtlesim_node
```

Voir les service ROS en cours
```bash
rosservice list
```