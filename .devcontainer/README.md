# ROS 2 Jazzy & OpenCV Devcontainer

Deze devcontainer is opgezet voor C++ ontwikkeling met ROS 2 Jazzy en OpenCV. Het biedt een geïsoleerde en reproduceerbare omgeving met alle benodigde tools en configuraties.

## Doel

Deze devcontainer is ontworpen om een naadloze ontwikkelervaring te bieden voor projecten die gebruikmaken van:
- **ROS 2 Jazzy**: De nieuwste Long-Term Support (LTS) release van het Robot Operating System.
- **OpenCV**: Een open-source computer vision bibliotheek.

De container is gebaseerd op de officiële `osrf/ros:jazzy-desktop-full-noble` Docker image, die een volledige desktop-installatie van ROS 2 bevat, inclusief tools zoals RViz en Gazebo.

## Configuratie

### Docker Image
- **Base Image**: `osrf/ros:jazzy-desktop-full-noble`
- **Gebruiker**: `ubuntu`

### VS Code Extensies
De volgende VS Code extensies worden automatisch geïnstalleerd in de container voor een betere ontwikkelervaring:
- `albert.tabout`: Voor het "tabben" uit haakjes en aanhalingstekens.
- `eamodio.gitlens`: Voor geavanceerde Git-integratie.
- `donjayamanne.githistory`: Voor het bekijken van de Git-geschiedenis.
- `ms-python.vscode-pylance`: Voor Pylance taalondersteuning in Python.
- `ms-python.python`: Voor Python-ondersteuning.
- `ms-vscode.cpptools`: Voor C/C++ taalondersteuning.
- `nonanonno.vscode-ros2`: Voor ROS 2-specifieke functionaliteit.
- `twxs.cmake`: Voor CMake-ondersteuning.

### Container Instellingen
- **Netwerk**: De container deelt de netwerk-namespace van de host (`--network=host`), wat essentieel is voor de communicatie tussen ROS 2 nodes.
- **GUI Applicaties**: De container is geconfigureerd om GUI-applicaties (zoals RViz of `cv::imshow`) te kunnen draaien door de X11-socket en DRI-apparaten van de host te mounten.
- **Workspace**: Je lokale projectmap wordt gemount in `/home/ubuntu/<jouw-project-naam>` in de container.
- **Gebruikersrechten**: De `ubuntu` gebruiker is toegevoegd aan de `video` groep om toegang te hebben tot webcams en andere video-apparaten.

## Hoe te gebruiken met een bestaand project

Volg deze stappen om deze devcontainer aan je eigen project toe te voegen:

1.  **Kloon je project repository:**
    ```bash
    cd <jouw-project-naam>
    git clone https://git.benjaminaarsen.nl/benjamin/ros-devcontainer.git .devcontainer
    ```

2.  **Open in VS Code:**
    Open de hoofdmap van je project in Visual Studio Code.

3.  **Heropen in Container:**
    VS Code zal detecteren dat er een `.devcontainer` map aanwezig is en een melding tonen in de rechteronderhoek. Klik op "Reopen in Container" om de devcontainer te bouwen en te starten.

Na het bouwen van de image, zal VS Code automatisch verbinden met de container en kun je beginnen met ontwikkelen in de geconfigureerde ROS 2 en OpenCV omgeving.

## Belangrijke opmerking: GUI Applicaties

Om GUI-applicaties (zoals `cv::imshow` of `rviz2`) vanuit de container op je host-machine te kunnen weergeven, maakt de devcontainer gebruik van de X11-socket van de host.

Na elke herstart van je **host-machine** moet je mogelijk het volgende commando in een terminal op je **host-machine** uitvoeren om de container toegang te geven tot de X-server:

```bash
xhost +
```

Dit commando zorgt ervoor dat de container vensters kan openen op je desktop. Zonder dit commando krijg je mogelijk foutmeldingen zoals "cannot open display".
