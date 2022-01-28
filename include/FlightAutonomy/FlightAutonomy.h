#pragma once

#include "FlightAutonomy/defines.h"

#include <opencv2/opencv.hpp>
#ifdef FA_DEBUG
#include <opencv2/highgui/highgui.hpp>
#endif
#include <chrono>

#include "FlightAutonomy/ImageReceiver.h"
#include "FlightAutonomy/FlightControl.h"
#include "FlightAutonomy/ObjectDetector.h"
#include "FlightAutonomy/algorithms.h"

/**
 * @brief Klasa odpowiedzialna za kompleksową obsługę autonomii lotu bazującej na analizie wizyjnej.
 * W klasie realizowany jest algorytm analizujący obraz odbierany z kamery i w przypadku wykrycia przeszkody wykonanie odpowiedniej reakcji w postaci zmiany trajektorii lotu maszyny.
 * Wykorzystywany jest protokół MavLink do dwukierunkowej komunikacji z autopilotem i przesyłania komend sterujących.
 */
class FlightAutonomy
{
#ifdef FA_DEBUG
    const std::string OPENCV_WINDOW = "Cam View";
#endif

    ImageReceiver imgRec;                                 ///< Odbiornik obrazu z kamery
    FlightControl flightCtrl;                             ///< Kontrola lotu maszyny
    ObjectDetector objDetect;                             ///< Wykrywacz obiektów na obrazie z kamery
    Algorithms currAlg;                                   ///< Obecnie wykonywany algorytm
    int landingPadID = 68;                                ///< ID znacznika Aruco lądowiska
    int gateArucos[4] = {10, 11, 12, 13};                 ///< ID znaczników umieszczonych na bramce zgodnie z ruchem wskazówek zegara
    int exitCode;                                         ///< Kod wyjścia z systemu: 0 - kontynuuj pracę, 1 - wyjście normalne, 2 - niepoprawna wysokość, 3 - nie wykryto znaczników przez zadany czas
    std::chrono::steady_clock::time_point timeoutCounter; ///< Odlicza czas do automatycznego wyjścia z programu w przypadku nie wykrywania znaczników

public:
    /**
     * @brief Konstruuje obiekt FlightAutonomy inicjalizując pola domyślnymi wartościami.
     */
    FlightAutonomy();

    /**
     * @brief Destuktor. W trybie debugowania niszczący okno OpenCV.
     */
    ~FlightAutonomy();

    /**
     * @brief Inicjalizuje połączenie z autopilotem poprzez protokół MavLink oraz uruchamia tryb offboard.
     *
     * @return true Połączenie zostało nawiązane.
     * @return false Wystąpił błąd podczas nawiązywania połączenia.
     */
    bool connect();

    /**
     * @brief Sprawdza czy maszyna jest gotowa do wykonywania algorytmu.
     *
     * @return true Maszyna jest gotowa.
     * @return false Maszyna nie jest gotowa.
     */
    bool isReady();

    /**
     * @brief Wczytuje przekazane do programu parametry.
     *
     * @param argc Liczba parametrów
     * @param argv Wskaźnik na tablicę parametrów
     *
     * @return true Poprawnie wczytano parametry.
     * @return false Nie można wczytać parametrów.
     */
    bool readArgs(const int argc, char **argv);

    /**
     * @brief Wykonuje pojedynczy krok algorytmu.
     * Powinna być wywoływana jednokrotnie w trakcie każdego obrotu pętli głównej programu.
     */
    bool spinOnce();

    /**
     * @brief Sprawdza czy wszystkie elementy działają prawidłowo i czy nie pojawił się warunek wyjścia.
     *
     * @return true Wszystkie komponenty działają prawidłowo i nie pojawił się warunek wyjścia.
     * @return false Pojawił się błąd działania lub warunek wyjścia.
     */
    bool ok();

    /**
     * @brief Kończy działanie algorytmu i wyłącza tryb offboard.
     *
     * @return true Pomyślnie zakończono działanie.
     * @return false Pojawił się błąd podczas próby zakończenia działania.
     */
    bool stop();

    /**
     * @brief Zwraca wartość kodu wyjścia.
     *
     * @return int Wartość kodu wyjścia.
     */
    int getExitCode();

    /**
     * @brief Wyświetla status wyjścia według kodu w zmiennej exitCode.
     */
    void printExitStatus();

private:
    /**
     * @brief Wykonuje pojedyncza iterację dla wybranego algorytmu.
     *
     * @param img Najnowsza odebrana ramka obrazu.
     *
     * @return int Kod zwrócony przez algorytm.
     */
    int performStep(cv::Mat &img);

    /**
     * @brief Wykonuje jeden krok algorytmu lądowania.
     *
     * @param img Najnowsza ramka obrazu.
     *
     * @return true Poprawnie wykonano krok algorytmu.
     * @return false Błąd podczas wykonywania kroku algorytmu.
     */
    bool landingStep(cv::Mat &img);

    /**
     * @brief Oblicza prędkości danej iteracji algorytmu lądowania.
     *
     * @param img Przetwarzana ramka obrazu.
     * @param arucoPosition Pozycja znacznika lądowiska na obrazie.
     * @return mavsdk::Offboard::VelocityBodyYawspeed
     */
    mavsdk::Offboard::VelocityBodyYawspeed landingCalcVelo(cv::Mat &img, cv::Point2f arucoPosition);

    /**
     * @brief Wykonuje jeden krok algorytmu lądowania.
     *
     * @param img Najnowsza ramka obrazu.
     *
     * @return true Poprawnie wykonano krok algorytmu.
     * @return false Błąd podczas wykonywania kroku algorytmu.
     */
    bool avoidingStep(cv::Mat &img);

    /**
     * @brief Oblicza prędkości danej iteracji algorytmu przelotu przez bramkę, potrzebne do przelotu przez bramkę.
     *
     * @param img Przetwarzana ramka obrazu.
     * @param gatePosition Pozycja bramki na obrazie.
     * @param angle Kąt pochylenia bramki.
     * @return mavsdk::Offboard::VelocityBodyYawspeed
     */
    mavsdk::Offboard::VelocityBodyYawspeed avoidingCalcVelo(cv::Mat &img, cv::Point2f gatePosition, float angle);

    /**
     * @brief Sprawdza czy nastąpiło przekroczenie zdefiniowanych czasów maksymalnych.
     */
    void checkTimeouts();
};