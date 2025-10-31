
# MIDI DMX Button Remote Trigger System

__Wireless Remote Button Transmitter & Receiver for triggering lighting or sound consoles via MIDI and DMX signals__

This project originated while working as lighting technician for a magician duo who were using a “sonar” device in their stage show. The sonar had a light in front and was controlled with an IR remote on the sonar’s handle. During a scene the magician scanned for ghosts in the crowd and the sonar “reacted” to one of the audience members. Later it was refitted with an DMX light which then could be controlled from the lighting console. Sounds were triggered with the different lighting scenes in a Culab session from the lighting desk using MIDI.

Since the synchronisation between the magician’s acting and the light needing to change was solely depending on the lighting operator, I searched for a more hands-off and easier solution. So I came up with the idea of an remote control device. The transmitter should be fitted several buttons for the different sonar’s actions. A receiver could receive the signal and send DMX and/or MIDI triggers to the lighting console.

As I already had a great knowledge in microcontroller development boards like the ESP32 platform I chose a variant from this platform. Furthermore conveniently the ESP32 incorporates an intercommunication protocol called ESP-NOW which I used to handle the communication between the button remote device and the receiver.
