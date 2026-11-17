/**
 * @file FwUpdate.h
 * @brief A new master firmware over Wi-Fi: uploaded from the portal into the
 *        spare app slot, and kept only once it has run properly.
 *
 * The partition table has two app slots (app0, app1) and otadata; USB uploads
 * write app0 and reset otadata to it, so a USB flash always works afterwards
 * too. An update is written to the slot not running, checked, and made the one
 * to boot; the master restarts into it through masterBeforeRestart() (the CAN
 * TX pad latched recessive, as for every portal restart).
 *
 * The bootloader's rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, on in the
 * Arduino core) is used, not Arduino's default: a new firmware starts "on
 * trial", and any restart before it confirms itself - a crash, a watchdog, a
 * brownout, deep sleep - boots the previous one again. It confirms itself
 * after FW_CONFIRM_MS of running with the portal up (or just before a clean
 * deep sleep, the portal up): one that cannot bring the portal up is never
 * kept, so the way back in is never lost.
 *
 * Refused while moving (the speed above 2 km/h, as clearing codes is). An
 * image must start like an ESP32 image (0xE9) and carry FW_MARK, which only
 * this firmware has: a display's firmware is an ESP32-S3 image too and would
 * pass every other check. Each write is announced with masterBusStall(): a
 * flash write holds the CAN interrupt off.
 */
#pragma once
#include <Arduino.h>

/** Running this long with the portal up confirms a new firmware [ms]. */
static constexpr uint32_t FW_CONFIRM_MS = 30000;
/** An upload that has sent nothing for this long is given up [ms]. */
static constexpr uint32_t FW_STALL_MS = 30000;

/** @brief May an update start now? @p why says why not. */
bool fwUpdateAllowed(char *why, size_t cap);
/** @brief An upload starts. false: refused, @p why says why. */
bool fwUpdateBegin(char *why, size_t cap);
/** @brief One piece of it. false: stopped (nothing changed), @p why says why. */
bool fwUpdateWrite(const uint8_t *data, size_t len, char *why, size_t cap);
/**
 * @brief The upload is complete: check it and make it the one to boot.
 * @return true = restart now (the caller does, after answering the page);
 *         false = nothing changed, @p why says why.
 */
bool fwUpdateEnd(char *why, size_t cap);
/** @brief The upload was cut off: nothing changes. */
void fwUpdateAbort();
/** @brief An upload is in progress (sleep waits for it). */
bool fwUpdateBusy();

/**
 * @brief From loop(), and with @p beforeSleep just before deep sleep: confirm
 *        a firmware on trial once it has run properly, and give up an upload
 *        that stalled.
 * @param portalUp The portal is up, or switched off by the settings.
 */
void fwUpdateTick(bool portalUp, bool beforeSleep = false);

/** @brief Say at boot which firmware this is, and if the last update failed. */
void fwBootReport();

/** @brief What the portal shows. */
struct FwStatus {
    const char *version;     /**< FIRMWARE_VERSION.                          */
    char        build[17];   /**< This image's ELF SHA-256, first 16 digits. */
    const char *slot;        /**< The app slot running ("app0", "app1").    */
    bool        trial;       /**< A new firmware not yet confirmed.          */
    bool        rolledBack;  /**< The last update did not run: back on this. */
    bool        busy;        /**< An upload in progress...                   */
    uint32_t    received;    /**< ...and how much of it has come.            */
    const char *mark;        /**< FW_MARK, the text every master image has.  */
};
void fwStatus(FwStatus &out);
