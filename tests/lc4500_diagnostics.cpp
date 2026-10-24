#include "lc4500tools.h"
#include "ProjectorLightCrafter4500.h"
#include "LC4500API/dlpc350_api.h"
#include "LC4500API/dlpc350_usb.h"
#include "LC4500API/dlpc350_firmware.h"
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <thread>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  try {
    require(argc == 2, "Usage: slstudio_lc4500_diagnostics --lc4500-usb|--lc4500-flash-header|--lc4500-flash-layout|--lc4500-bootloader|--lc4500-recover-bootloader");
    const bool flashHeader = std::string(argv[1]) == "--lc4500-flash-header";
    const bool flashLayout = std::string(argv[1]) == "--lc4500-flash-layout" || flashHeader;
    const bool bootloader = std::string(argv[1]) == "--lc4500-bootloader";
    const bool recoverBootloader = std::string(argv[1]) == "--lc4500-recover-bootloader";
    require(argc == 2 && (std::string(argv[1]) == "--lc4500-usb" || flashLayout || bootloader || recoverBootloader),
        "Unknown projector test argument");
    require(DLPC350_USB_Init() == 0, "HID initialization failed");
    struct ConnectionGuard {
      ~ConnectionGuard() { DLPC350_USB_Close(); DLPC350_USB_Exit(); }
    } connection;
    require(DLPC350_USB_IsPresent() != 0, "No LC4500 HID device detected");
    require(DLPC350_USB_Open() == 0, "Cannot open LC4500; check hidraw permissions or another application's device access");
    if (recoverBootloader) {
      unsigned char status = 0;
      require(DLPC350_GetBLStatus(&status) >= 0 && (status & 0x80) != 0,
          "Device is not in the bootloader; no exit command was sent");
      require(DLPC350_ExitProgrammingMode() >= 0, "Bootloader recovery exit failed");
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      do {
        DLPC350_USB_Close();
        unsigned int app = 0, api = 0, software = 0, sequence = 0;
        if (DLPC350_USB_Open() >= 0 && DLPC350_GetVersion(&app, &api, &software, &sequence) >= 0 &&
            app == 0x04040000) {
          std::cout << "Recovered application firmware " << slstudio::lightCrafter4500VersionString(app)
                    << "; no erase/program commands sent\n";
          return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while (std::chrono::steady_clock::now() < deadline);
      throw std::runtime_error("Bootloader recovery did not return to application firmware");
    }
    unsigned int application = 0, api = 0, software = 0, sequence = 0;
    require(DLPC350_GetVersion(&application, &api, &software, &sequence) == 0, "LC4500 firmware version query failed");
    bool standby = false;
    require(DLPC350_GetPowerMode(&standby) == 0, "LC4500 power status query failed");
    std::cout << "LC4500 HID communication passed; firmware 0x" << std::hex << application
              << ", API 0x" << api << std::dec << ", standby " << standby << '\n';
    if (bootloader) {
      require(application == 0x04040000, "Bootloader probe supports firmware 4.4.0 only");
      const auto expectedVersion = application;
      std::uint32_t expectedChecksum = 0;
      for (unsigned int offset = 0; offset < sizeof(FLASH_TABLE); offset += 4) {
        unsigned int value = 0;
        require(DLPC350_MemRead(FLASH_BASE_ADDRESS + 128 * 1024 + offset, &value) >= 0,
            "Cannot read metadata header before checksum probe");
        for (unsigned int byte = 0; byte < 4; ++byte)
          expectedChecksum += (value >> (byte * 8)) & 255;
      }
      const auto reconnect = [&](bool boot) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        do {
          DLPC350_USB_Close();
          if (DLPC350_USB_Open() >= 0) {
            unsigned char status = 0;
            const bool ready = boot
                ? DLPC350_GetBLStatus(&status) >= 0 && (status & 0x80) != 0
                : DLPC350_GetVersion(&application, &api, &software, &sequence) >= 0 && application == expectedVersion;
            if (ready) return;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } while (std::chrono::steady_clock::now() < deadline);
        throw std::runtime_error(boot ? "Bootloader connection timed out" : "Application reconnect timed out");
      };
      bool entered = false;
      try {
        entered = true;
        const auto result = DLPC350_EnterProgrammingMode();
        std::cout << "Enter-programming command result: " << result << std::endl;
        require(result >= 0, "Enter-programming command was not acknowledged");
        reconnect(true);
        unsigned short manufacturer = 0;
        unsigned long long flashDevice = 0;
        require(DLPC350_GetFlashManID(&manufacturer) >= 0, "Flash manufacturer query failed");
        require(DLPC350_GetFlashDevID(&flashDevice) >= 0, "Flash device query failed");
        std::cout << "Bootloader connected; manufacturer 0x" << std::hex << manufacturer
                  << ", device 0x" << flashDevice << std::dec << std::endl;
        require(manufacturer == 0x20 && flashDevice == 0x227e, "Unsupported flash chip for checksum probe");
        require(DLPC350_SetFlashType(0) >= 0 && DLPC350_SetFlashAddr(128 * 1024) >= 0 &&
            DLPC350_SetUploadSize(sizeof(FLASH_TABLE)) >= 0 && DLPC350_CalculateFlashChecksum() >= 0,
            "Read-only checksum request failed");
        const auto checksumDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        for (;;) {
          unsigned char status = 0;
          require(DLPC350_GetBLStatus(&status) >= 0, "Checksum status query failed");
          if ((status & STAT_BIT_FLASH_BUSY) == 0) break;
          require(std::chrono::steady_clock::now() < checksumDeadline, "Checksum calculation timed out");
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        unsigned int actualChecksum = 0;
        require(DLPC350_GetFlashChecksum(&actualChecksum) >= 0, "Checksum read failed");
        std::cout << "Metadata header checksum: device 0x" << std::hex << actualChecksum
                  << ", expected 0x" << expectedChecksum << std::dec << std::endl;
        require(actualChecksum == expectedChecksum, "Read-only flash checksum differs from metadata readback");
        require(DLPC350_ExitProgrammingMode() >= 0, "Exit-programming command failed");
        reconnect(false);
        entered = false;
        std::cout << "Returned to application firmware " << slstudio::lightCrafter4500VersionString(application)
                  << "; no erase/program commands sent\n";
      } catch (...) {
        if (entered) {
          if (DLPC350_USB_Open() < 0 || DLPC350_ExitProgrammingMode() < 0)
            std::cerr << "Could not request bootloader exit; power-cycle may be necessary\n";
          else {
            try { reconnect(false); std::cerr << "Recovered application connection after probe failure\n"; }
            catch (const std::exception &error) { std::cerr << "Recovery: " << error.what() << '\n'; }
          }
        }
        throw;
      }
    }
    if (flashLayout) {
      require(application == 0x04040000, "Read-only flash probe supports firmware 4.4.0 only");
      constexpr unsigned int sectorSize = 128 * 1024;
      constexpr unsigned int address = FLASH_BASE_ADDRESS + sectorSize;
      unsigned int signature = 0;
      const auto result = DLPC350_MemRead(address, &signature);
      std::cout << "Flash-table read at 0x" << std::hex << address << ": result " << std::dec << result
                << ", value 0x" << std::hex << signature << std::dec << std::endl;
      require(result >= 0, "Controller refused flash memory read; no erase/program commands were sent");
      require(signature == FLASHTABLE_APP_SIGNATURE,
          "Flash-table signature mismatch; no erase/program commands were sent");
      const auto bytes = flashHeader ? static_cast<unsigned int>(sizeof(FLASH_TABLE)) : sectorSize;
      std::vector<unsigned int> words(bytes / 4);
      words[0] = signature;
      for (unsigned int index = 1; index < words.size(); ++index) {
        require(DLPC350_MemRead(address + index * 4, &words[index]) >= 0,
            "Flash sector read failed; no erase/program commands were sent");
        if (index % 4096 == 0)
          std::cout << "Read " << index * 4 << " / " << sectorSize << " metadata bytes" << std::endl;
      }
      for (unsigned int index = 0; index < words.size(); index += 64) {
        unsigned int repeated = 0;
        require(DLPC350_MemRead(address + index * 4, &repeated) >= 0 && repeated == words[index],
            "Flash sector readback was not stable; no erase/program commands were sent");
      }
      std::cout << "Read-only flash probe passed: " << bytes
                << " bytes, stable samples every 256 bytes; splash address 0x"
                << std::hex << words[offsetof(FLASH_TABLE, Splash_Data) / 4] << std::dec << '\n';
      std::cout << "Table version 0x" << std::hex << words[2] << ", boot 0x" << words[1]
                << ", free area 0x" << words[3] << '\n';
      for (unsigned int index = 4; index < sizeof(FLASH_TABLE) / 4; index += 2)
        if (words[index] != 0 && words[index] != 0xffffffff)
          std::cout << "Block at table offset 0x" << index * 4 << ": address 0x"
                    << words[index] << ", bytes 0x" << words[index + 1] << '\n';
      std::cout << std::dec;
      std::vector<unsigned char> snapshot(bytes);
      for (std::size_t index = 0; index < words.size(); ++index)
        for (unsigned int byte = 0; byte < 4; ++byte)
          snapshot[index * 4 + byte] = static_cast<unsigned char>(words[index] >> (byte * 8));
      const auto plan = slstudio::prepareLightCrafter4500FlashUpdate(application, snapshot,
          {cv::Mat3b(1140, 912, cv::Vec3b(42, 42, 42))});
      std::cout << "Read-only upload plan: " << plan.patternData.size() << " pattern bytes at flash offset 0x"
                << std::hex << plan.patternOffset << std::dec << "; table-sector rewrite "
                << (plan.tableSector != snapshot ? "required (full sector backup needed)" : "not required")
                << "; no erase/program commands sent\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
