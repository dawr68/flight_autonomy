#pragma once

#include <future>
#include <chrono>
#include <iostream>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include "defines.h"
#include "TelemetryData.h"

/**
 * @brief Klasa odpowiedzialna za komunikację z autopilotem poprzez protoków MAVLink.
 * Zapewnia odbiór kluczowych danych telemetrycznych i pozwala na sterowanie poprzez komendy offboard.
 */
class FlightControl
{
    std::string connectionURL; ///< Adres połączenia protokołu MAVLink

    mavsdk::Mavsdk mavsdk;                        ///< Pozwala na zarządzanie połączeniami
    std::shared_ptr<mavsdk::System> system;       ///< Obiekt reprezentujący system (autopilot)
    std::shared_ptr<mavsdk::Info> info;           ///< Dostarcza informacji o systemie
    std::shared_ptr<mavsdk::Telemetry> telemetry; ///< Pozwala na odbiór danych telemetrycznych
    std::shared_ptr<mavsdk::Action> action;       ///< Pozwala na wykonywanie prostych akcji
    std::shared_ptr<mavsdk::Offboard> offboard;   ///< Pozwala na kontrolę w trybie offboard

    TelemetryData telemData; ///< Przechowuje dane telemetryczne odebrane przez MAVLink

    /**
     * @brief Zwraca obecny system
     *
     * @return std::shared_ptr<mavsdk::System> Obecny system
     */
    std::shared_ptr<mavsdk::System> getSystem();

    /**
     * @brief Subskrybuje wymagane tematy
     *
     */
    void subscribeTelem();

public:
    /**
     * @brief Tworzy nowy obiekt klasy FlightControl.
     */
    FlightControl() = default;

    /**
     * @brief Niszczy dany obiekt klasy FlightControl.
     */
    ~FlightControl() = default;

    /**
     * @brief Inicjalizuje połączenie z autopilotem.
     *
     * @return true Pomyślnie nawiązano połącznie.
     * @return false Wystąpił błąd podczan nawiązywania połączenia.
     */
    bool connect();

    /**
     * @brief Sprawdza stan maszyny pod kątem gotowości do działania systemu.
     *
     * @return true Maszyna jest gotowa do wykonywania algorytmu.
     * @return false Maszyna nie jest gotowa do wykonywania algorytmu.
     */
    bool checkStatus();

    /**
     * @brief Zwraca aktualne kąty pochylenia maszyny.
     *
     * @return mavsdk::Telemetry::EulerAngle Kąty Eulera
     */
    mavsdk::Telemetry::EulerAngle getEulerAngle();

    /**
     * @brief Ustawia wartość pola connectionURL.
     *
     * @param url Nowy url do ustawienia.
     */
    void setConnectionURL(const std::string url);

    /**
     * @brief Wyświetla w konsoli podstawowe dane telemetryczne.
     */
    void printTelem();

    /**
     * @brief Sprawdzenie czy maszyna jest w trakcie lotu.
     *
     * @return true Maszyna jest w trakcie lotu.
     * @return false Maszyna nie jest w trakcie lotu.
     */
    bool observeInAir();

    /**
     * @brief Aktywuje kontrolę w trybie offboard i ustawia prędkości ciała na 0.
     *
     * @return true Pomyślnie aktywowano kontrolę offboard.
     * @return false Wystąpił błąd podczas aktywowania kontroli offboard.
     */
    bool startOffbard();

    /**
     * @brief Zatrzymuje kontrolę w trybie offboard.
     *
     * @return true Pomyślnie zatrzymano kontrolę offboard.
     * @return false Wystąpił błąd podczas zatrzymywania kontroli offboard.
     */
    bool stopOffboard();

    /**
     * @brief Ustawia prędkości liniowe dla ciała w trybie offboard.
     *
     * @return true Pomyślnie ustawiono prędkość.
     * @return false Wystąpił błąd podczas ustawiania prędkości.
     */
    bool setOffbardVelo(mavsdk::Offboard::VelocityBodyYawspeed veloBodyYawspeed);

    /**
     * @brief Wyzwala tryb lądowania w obecnym punkcie.
     *
     * @return true Pomyślnie aktywowano tryb lądowanie.
     * @return false Wystąpił błąd podczas aktywowania trybu lądowania.
     */
    bool land();

    /**
     * @brief Zwraca ostatnią wysokość relatywną maszyny nad ziemią.
     *
     * @return float Wysokość AGL maszyny.
     */
    float getAltitude();

    /**
     * @brief Zwraca aktualny tryb lotu.
     *
     * @return Aktualny tryb lotu.
     */
    mavsdk::Telemetry::FlightMode getFlightMode();

    /**
     * @brief Sprawdza czy zadane prędkości mieszczą się w dopuszczalnych wartościach i jeżeli nie mieszczą się, wprowadza korektę.
     *
     * @param velocities Prędkości podlegające sprawdzeniu.
     * @return true Wartości prędkości mieszczą się w zadanym przedziale.
     * @return false Wartości prędkości nie mieszczą się w zadanym przedziale, wprowadzona zastała korekta.
     */
    bool checkVelo(mavsdk::Offboard::VelocityBodyYawspeed &velocities);
};