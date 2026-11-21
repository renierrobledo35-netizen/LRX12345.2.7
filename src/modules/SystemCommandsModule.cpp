#include "SystemCommandsModule.h"
#include "input/InputBroker.h"
#include "meshUtils.h"
#include "Router.h"

#if HAS_SCREEN
#include "MessageStore.h"
#include "graphics/Screen.h"
#include "graphics/SharedUIDisplay.h"
#endif

#include "GPS.h"
#include "RTC.h"
#include "MeshService.h"
#include "Module.h"
#include "NodeDB.h"
#include "main.h"
#include "modules/AdminModule.h"
#include "modules/ExternalNotificationModule.h"

SystemCommandsModule *systemCommandsModule;
// LoRaCTS: Send SOS with phone GPS first, then onboard GPS fallback
static void sendLoRaCTSSOS()
{
    char message[128];
    uint32_t now = getValidTime(RTCQualityFromNet);

    // Priority 1: Recent phone GPS coordinates
    bool phoneGPSValid =
        now > 0 &&
        localPosition.location_source == meshtastic_Position_LocSource_LOC_EXTERNAL &&
        (localPosition.latitude_i != 0 || localPosition.longitude_i != 0) &&
        localPosition.time > 0 &&
        localPosition.time <= now &&
        (now - localPosition.time) <= 180;

    if (phoneGPSValid) {
        snprintf(message, sizeof(message),
                 "SOS! Help needed. %.7f, %.7f",
                 (double)localPosition.latitude_i / 10000000.0,
                 (double)localPosition.longitude_i / 10000000.0);
    }
    // Priority 2: Onboard NEO-6M GPS
    else if (gps != nullptr && gps->hasLock() &&
             (gps->p.latitude_i != 0 || gps->p.longitude_i != 0)) {
        snprintf(message, sizeof(message),
                 "SOS! Help needed. %.7f, %.7f",
                 (double)gps->p.latitude_i / 10000000.0,
                 (double)gps->p.longitude_i / 10000000.0);
    }
    // Priority 3: No valid GPS source
    else {
        snprintf(message, sizeof(message), "SOS! Help needed.");
    }

    meshtastic_MeshPacket *p = router->allocForSending();

    if (!p) {
        LOG_ERROR("LoRaCTS SOS: Packet allocation failed");
        return;
    }

    p->to = NODENUM_BROADCAST;
    p->channel = 0;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->decoded.payload.size = strlen(message);

    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);

    p->want_ack = false;
    p->decoded.want_response = false;

    service->sendToMesh(p, RX_SRC_LOCAL, true);

    LOG_INFO("LoRaCTS SOS sent: %s", message);
}

SystemCommandsModule::SystemCommandsModule()
{
    if (inputBroker)
        inputObserver.observe(inputBroker);
}

int SystemCommandsModule::handleInputEvent(const InputEvent *event)
{
    LOG_INPUT("SystemCommands Input event %u! kb %u", event->inputEvent, event->kbchar);
    // System commands (all others fall through)
    switch (event->kbchar) {
    // Fn key symbols
    case INPUT_BROKER_MSG_FN_SYMBOL_ON:
    case INPUT_BROKER_MSG_FN_SYMBOL_OFF:
        return 0;
    // Brightness
    case INPUT_BROKER_MSG_BRIGHTNESS_UP:
        IF_SCREEN(screen->increaseBrightness());
        LOG_DEBUG("Increase Screen Brightness");
        return 0;
    case INPUT_BROKER_MSG_BRIGHTNESS_DOWN:
        IF_SCREEN(screen->decreaseBrightness());
        LOG_DEBUG("Decrease Screen Brightness");
        return 0;
    // Mute
    case INPUT_BROKER_MSG_MUTE_TOGGLE:
        if (moduleConfig.external_notification.enabled && externalNotificationModule) {
            externalNotificationModule->setMute(!externalNotificationModule->getMute());
            IF_SCREEN(if (!externalNotificationModule->getMute()) externalNotificationModule->stopNow(); screen->showSimpleBanner(
                externalNotificationModule->getMute() ? "Notifications\nDisabled" : "Notifications\nEnabled", 3000);)
        }
        return 0;
    // Bluetooth
    case INPUT_BROKER_MSG_BLUETOOTH_TOGGLE:
        config.bluetooth.enabled = !config.bluetooth.enabled;
        LOG_INFO("User toggled Bluetooth");
        nodeDB->saveToDisk();
#if defined(ARDUINO_ARCH_NRF52)
        if (!config.bluetooth.enabled) {
            disableBluetooth();
            IF_SCREEN(screen->showSimpleBanner("Bluetooth OFF\nRebooting", 3000));
            rebootAtMsec = millis() + DEFAULT_REBOOT_SECONDS * 2000;
        } else {
            IF_SCREEN(screen->showSimpleBanner("Bluetooth ON\nRebooting", 3000));
            rebootAtMsec = millis() + DEFAULT_REBOOT_SECONDS * 1000;
        }
#else
        if (!config.bluetooth.enabled) {
            disableBluetooth();
            IF_SCREEN(screen->showSimpleBanner("Bluetooth OFF", 3000));
        } else {
            IF_SCREEN(screen->showSimpleBanner("Bluetooth ON\nRebooting", 3000));
            rebootAtMsec = millis() + DEFAULT_REBOOT_SECONDS * 1000;
        }
#endif
        return 0;
    case INPUT_BROKER_MSG_REBOOT:
        IF_SCREEN(screen->showSimpleBanner("Rebooting...", 0));
        nodeDB->saveToDisk();
#if HAS_SCREEN
        messageStore.saveToFlash();
#endif
        rebootAtMsec = millis() + DEFAULT_REBOOT_SECONDS * 1000;
        // runState = CANNED_MESSAGE_RUN_STATE_INACTIVE;
        return true;
    }

    switch (event->inputEvent) {
        //LoRacts SOS Button
    case INPUT_BROKER_LORACTS_SOS:
        LOG_INFO("LoRaCTS SOS button pressed");
        sendLoRaCTSSOS();
        return true;
        // GPS
    case INPUT_BROKER_GPS_TOGGLE:
#if !MESHTASTIC_EXCLUDE_GPS
        if (gps) {
            if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_ENABLED &&
                config.position.fixed_position == false) {
                nodeDB->clearLocalPosition();
                nodeDB->saveToDisk();
            }
            gps->toggleGpsMode();
            const char *msg =
                (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_ENABLED) ? "GPS Enabled" : "GPS Disabled";
            IF_SCREEN(screen->forceDisplay(); screen->showSimpleBanner(msg, 3000);)
        }
#endif
        return true;
    // Mesh ping
    case INPUT_BROKER_SEND_PING:
        service->refreshLocalMeshNode();
        if (service->trySendPosition(NODENUM_BROADCAST, true)) {
            IF_SCREEN(screen->showSimpleBanner("Position\nSent", 3000));
        } else {
            IF_SCREEN(screen->showSimpleBanner("Node Info\nSent", 3000));
        }
        return true;
    // Power control
    case INPUT_BROKER_SHUTDOWN:
        shutdownAtMsec = millis();
        return true;
    // factory reset
    case INPUT_BROKER_FACTORY_RST:
        disableBluetooth();
        LOG_INFO("Initiate full factory reset");
        nodeDB->factoryReset(true);
        // reboot(DEFAULT_REBOOT_SECONDS);
        LOG_INFO("Reboot in %d seconds", DEFAULT_REBOOT_SECONDS);
        if (screen)
            screen->showSimpleBanner("Rebooting...", 0); // stays on screen
        rebootAtMsec = (DEFAULT_REBOOT_SECONDS < 0) ? 0 : (millis() + DEFAULT_REBOOT_SECONDS * 1000);
        return true;

    default:
        // No other input events handled here
        break;
    }
    return false;
}
