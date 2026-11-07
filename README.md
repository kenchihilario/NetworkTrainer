# NetworkTrainer

a 3D demo of our network fault trainer. you can cut a cable, peek inside the box, and see what happens to each laptop. made for our school project at PUP.

live demo: https://networktrainer.vercel.app

(it needs a browser with WebGL and internet, so don't open it on a potato. sorry)

## what is this?

our project is an ESP32-based, remote-controlled Ethernet fault injection trainer. basically a box with 3 laptops connected to it, and the box can "break" one of the cables on purpose so students can practice physical network troubleshooting without anyone having to unplug things by hand.

this repo is the website version, so people can see what we propose it to look like before it's even built.

## what can you do on the page?

- **Cut cable:** pick Laptop 1, 2, or 3 and that cable gets "cut". pick None to fix everything.
- **X-ray:** shows the inside of the box (relays, wires, etc.)
- **Zoom in / Zoom out:** self explanatory
- **drag** to rotate the model

## files

| file | what it is |
|------|-----------|
| `index.html` | the web page (uses Three.js) |
| `netfault_trainer.glb` | the 3D model, exported from Blender |
| `netfault_trainer_scene.py` | the Blender script that builds the whole scene |

## how to run it on your own computer

you can't just double click `index.html`. the browser blocks the model from loading (took us a while to figure that out). do this instead:

1. download or clone this repo
2. open a terminal in the folder
3. run `python -m http.server 8000`
4. open http://localhost:8000

if you see "couldn't load netfault_trainer.glb", the .glb is missing, renamed, or you opened the file directly instead of through the server. press F12 and check the Console for the real error.

## how to remake the 3D model

1. open Blender (we used a newer version than the script was written for, it still worked for us)
2. go to the Scripting tab and open `netfault_trainer_scene.py`
3. press Alt+P to run it
4. `BAKE_FOR_WEB = True` makes it look nicer but takes a few minutes and Blender will look frozen. just wait.
5. it saves `netfault_trainer.glb` in your home folder. put it next to `index.html`.

## made with

- Blender (3D model)
- Three.js r128 + GLTFLoader (the web page)
- Vercel (hosting)

## notes / known problems

- not tested on every browser or phone
- colors of some metal parts might look a bit dark depending on your screen
- big .glb file means slow loading on bad wifi

## made by

Kenchi Paul M. Hilario
Polytechnic University of the Philippines
