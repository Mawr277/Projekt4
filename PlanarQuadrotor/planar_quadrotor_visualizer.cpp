#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}


void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    // -----------------------------------------------
    // Sekcja: Rysowanie ramion quadrotora
    // -----------------------------------------------

    int arm_length = 60;
    int arm_x1 = static_cast<int>(q_x + arm_length * cos(q_theta));
    int arm_y1 = static_cast<int>(q_y + arm_length * sin(q_theta));
    int arm_x2 = static_cast<int>(q_x - arm_length * cos(q_theta));
    int arm_y2 = static_cast<int>(q_y - arm_length * sin(q_theta));

    // Ustawienie koloru rysowania na zielony i rysowanie linii ramienia
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0xFF, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), arm_x1, arm_y1, arm_x2, arm_y2);

    // -----------------------------------------------
    // Sekcja: Rysowanie prostok�ta reprezentuj�cego cia�o quadrotora
    // -----------------------------------------------
    int rect_width = 60;
    int rect_height = 30;

    // Definicja wierzcho�k�w prostok�ta w lokalnym uk�adzie wsp�rz�dnych
    SDL_Point corners[4] = {
        {-rect_width / 2, -rect_height / 2},
        {rect_width / 2, -rect_height / 2},
        {rect_width / 2, rect_height / 2},
        {-rect_width / 2, rect_height / 2}
    };

    // Transformacja wierzcho�k�w do globalnego uk�adu wsp�rz�dnych
    for (int i = 0; i < 4; i++) {
        // Obr�t i przesuni�cie wierzcho�ka
        float x = corners[i].x * cos(q_theta) - corners[i].y * sin(q_theta);
        float y = corners[i].x * sin(q_theta) + corners[i].y * cos(q_theta);
        // Przesuni�cie do globalnego uk�adu wsp�rz�dnych
        corners[i].x = static_cast<int>(x + q_x);
        corners[i].y = static_cast<int>(y + q_y);
    }

    // Ustawienie koloru rysowania na czerwony
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);

    // Przygotowanie tablic wsp�rz�dnych do rysowania prostok�ta
    Sint16 vx[4];
    Sint16 vy[4];

    for (int i = 0; i < 4; ++i) {
        // Przypisanie przetransformowanych wsp�rz�dnych do tablic
        vx[i] = corners[i].x;
        vy[i] = corners[i].y;
    }

    // Rysowanie wype�nionego prostok�ta na ekranie
    filledPolygonColor(gRenderer.get(), vx, vy, 4, 0xFF0000FF);

    // -----------------------------------------------
    // Sekcja: Rysowanie wirnik�w quadrotora
    // -----------------------------------------------
    int propeller_radius = 10;

    // Rysowanie wirnik�w jako zielone ko�a
    filledCircleColor(gRenderer.get(), arm_x1, arm_y1, propeller_radius, 0x00FF00FF);
    filledCircleColor(gRenderer.get(), arm_x2, arm_y2, propeller_radius, 0x00FF00FF);

    // Animacja wirnik�w
    float propeller_angle = SDL_GetTicks() / 50.0;

    // Obliczanie wsp�rz�dnych ko�c�wek ramion wirnik�w
    int prop_arm_x1 = static_cast<int>(arm_x1 + propeller_radius * cos(propeller_angle));
    int prop_arm_y1 = static_cast<int>(arm_y1 + propeller_radius * sin(propeller_angle));
    int prop_arm_x2 = static_cast<int>(arm_x1 - propeller_radius * cos(propeller_angle));
    int prop_arm_y2 = static_cast<int>(arm_y1 - propeller_radius * sin(propeller_angle));

    int prop_arm_x3 = static_cast<int>(arm_x2 + propeller_radius * cos(propeller_angle));
    int prop_arm_y3 = static_cast<int>(arm_y2 + propeller_radius * sin(propeller_angle));
    int prop_arm_x4 = static_cast<int>(arm_x2 - propeller_radius * cos(propeller_angle));
    int prop_arm_y4 = static_cast<int>(arm_y2 - propeller_radius * sin(propeller_angle));

    // Ustawienie koloru rysowania na niebieski i rysowanie ramion wirnik�w
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), prop_arm_x1, prop_arm_y1, prop_arm_x2, prop_arm_y2);
    SDL_RenderDrawLine(gRenderer.get(), prop_arm_x3, prop_arm_y3, prop_arm_x4, prop_arm_y4);


}