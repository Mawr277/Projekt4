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
    // Sekcja: Rysowanie prostok¹ta reprezentuj¹cego cia³o quadrotora
    // -----------------------------------------------
    int rect_width = 60;
    int rect_height = 30;

    // Definicja wierzcho³ków prostok¹ta w lokalnym uk³adzie wspó³rzêdnych
    SDL_Point corners[4] = {
        {-rect_width / 2, -rect_height / 2},
        {rect_width / 2, -rect_height / 2},
        {rect_width / 2, rect_height / 2},
        {-rect_width / 2, rect_height / 2}
    };

    // Transformacja wierzcho³ków do globalnego uk³adu wspó³rzêdnych
    for (int i = 0; i < 4; i++) {
        // Obrót i przesuniêcie wierzcho³ka
        float x = corners[i].x * cos(q_theta) - corners[i].y * sin(q_theta);
        float y = corners[i].x * sin(q_theta) + corners[i].y * cos(q_theta);
        // Przesuniêcie do globalnego uk³adu wspó³rzêdnych
        corners[i].x = static_cast<int>(x + q_x);
        corners[i].y = static_cast<int>(y + q_y);
    }

    // Ustawienie koloru rysowania na czerwony
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);

    // Przygotowanie tablic wspó³rzêdnych do rysowania prostok¹ta
    Sint16 vx[4];
    Sint16 vy[4];

    for (int i = 0; i < 4; ++i) {
        // Przypisanie przetransformowanych wspó³rzêdnych do tablic
        vx[i] = corners[i].x;
        vy[i] = corners[i].y;
    }

    // Rysowanie wype³nionego prostok¹ta na ekranie
    filledPolygonColor(gRenderer.get(), vx, vy, 4, 0xFF0000FF);

    // -----------------------------------------------
    // Sekcja: Rysowanie wirników quadrotora
    // -----------------------------------------------
    int propeller_radius = 10;

    // Rysowanie wirników jako zielone ko³a
    filledCircleColor(gRenderer.get(), arm_x1, arm_y1, propeller_radius, 0x00FF00FF);
    filledCircleColor(gRenderer.get(), arm_x2, arm_y2, propeller_radius, 0x00FF00FF);

    // Animacja wirników
    float propeller_angle = SDL_GetTicks() / 50.0;

    // Obliczanie wspó³rzêdnych koñcówek ramion wirników
    int prop_arm_x1 = static_cast<int>(arm_x1 + propeller_radius * cos(propeller_angle));
    int prop_arm_y1 = static_cast<int>(arm_y1 + propeller_radius * sin(propeller_angle));
    int prop_arm_x2 = static_cast<int>(arm_x1 - propeller_radius * cos(propeller_angle));
    int prop_arm_y2 = static_cast<int>(arm_y1 - propeller_radius * sin(propeller_angle));

    int prop_arm_x3 = static_cast<int>(arm_x2 + propeller_radius * cos(propeller_angle));
    int prop_arm_y3 = static_cast<int>(arm_y2 + propeller_radius * sin(propeller_angle));
    int prop_arm_x4 = static_cast<int>(arm_x2 - propeller_radius * cos(propeller_angle));
    int prop_arm_y4 = static_cast<int>(arm_y2 - propeller_radius * sin(propeller_angle));

    // Ustawienie koloru rysowania na niebieski i rysowanie ramion wirników
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), prop_arm_x1, prop_arm_y1, prop_arm_x2, prop_arm_y2);
    SDL_RenderDrawLine(gRenderer.get(), prop_arm_x3, prop_arm_y3, prop_arm_x4, prop_arm_y4);


}