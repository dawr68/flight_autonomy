#pragma once
#include "FlightAutonomy/defines.h"

#include <tuple>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

/**
 * @brief Klasa odpowiedzialna za analizę i wykrywanie oraz określanie pozycji obiektów (w tym znaczników Aruco) znajdujących się na przekazanym obrazie.
 *
 */
class ObjectDetector
{
    std::vector<int> markIds;                       /**< Wektor wszystkich wykrytych na obrazie znaczników. */
    std::vector<std::vector<cv::Point2f>> markCor;  /**< Wektor wektorów koordynatów każdego wykrytecho znacznika. */
    std::vector<std::vector<cv::Point2f>> rejected; /**< Wektor odrzuconych potencjalnych obiektów będących znacznikami. */

    cv::Ptr<cv::aruco::DetectorParameters> params; /**< Wskaźnik na parametry algorytmu wykrywającego znaczniki. */
    cv::Ptr<cv::aruco::Dictionary> dict;           /**< Wskaźnik na słownik znaczników. */

public:
    /**
     * @brief Konstruuje obiekt klasy ObjectDetector inicjalizując listę parametrów i słownik wartościami domyślnymi.
     */
    ObjectDetector();

    /**
     * @brief Konstruuje obiekt klasy ObjectDetector inicjalizując listę parametrów i słownik przekazanymi wartościami.
     *
     * @param _params Parametry algorytmu wykrywania znaczników.
     * @param _dict Słownik którym ma się posługiwać algorytm wykrywania znaczników.
     */
    ObjectDetector(cv::Ptr<cv::aruco::DetectorParameters> _params, cv::Ptr<cv::aruco::Dictionary> _dict);

    /**
     * @brief Destruktor domyślny.
     */
    ~ObjectDetector() = default;

    /**
     * @brief Wykrywa znaczniki Aruco na przekazanej ramce obrazu i oblicza koordynaty środka znacznika. Zwraca (-1, -1) jeżeli nie wykryto znacznika.
     *
     * @param img Ramka obrazu na której mają zostać wykryte znaczniki.
     * @param arucoID ID znacznika Aruco którego koordynaty mają zostać określone.
     * @return cv::Point2f Koordynaty środka znacznika Aruco.
     */
    cv::Point2f detectArucoSingle(const cv::Mat &img, int arucoID);

    /**
     * @brief Wykrywa bramkę składającą się z czterech znaczników i oblicza jej środek. Zwraca (-1, -1, -1) jeżeli nie wykryto bramki.
     *
     * @param img Ramka obrazu na której mają zostać wykryte znaczniki.
     * @param arucoIDs Tablica identyfikatorów kolejnych znaczników tworzących bramkę.
     * @return std::tuple<cv::Point2f, float, float> Koordynaty środka bramki, jej rozmiar, i pochylenie.
     */
    std::tuple<cv::Point2f, float, float> detectArucoGate(const cv::Mat &img, int arucoIDs[4]);

private:
    /**
     * @brief Oblicza środek znacznika aruco na podstawie przekazanego wektora koordynatów wierzchołków.
     *
     * @param corners Wektor zawierający koordynaty kolejnych wierzchołków znacznika.
     * @return cv::Point2f Obliczony środek znacznika.
     */
    cv::Point2f calcCenter(std::vector<cv::Point2f> corners);

    /**
     * @brief Oblicza względny rozmiar znacznika na ramce obrazu.
     * 
     * @param corners Wektor zawierający koordynaty kolejnych wierzchołków znacznika.
     * @param imgHeight Wysokość analizowanego obrazu.
     * @return float Względny rozmiar znacznika dla osi X i Y.
     */
    float calcMarkerSize(std::vector<cv::Point2f> corners, int imgHeight);

};