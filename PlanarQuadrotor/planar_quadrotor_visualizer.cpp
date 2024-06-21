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
    float arm_angle = M_PI / 6; 

    
    float arm_x1 = q_x + arm_length * cos(q_theta - arm_angle);
    float arm_y1 = q_y + arm_length * sin(q_theta - arm_angle);
    float arm_x2 = q_x - arm_length * cos(q_theta + arm_angle);
    float arm_y2 = q_y - arm_length * sin(q_theta + arm_angle);

    
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0xFF, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), arm_x1, arm_y1, q_x, q_y);
    SDL_RenderDrawLine(gRenderer.get(), q_x, q_y, arm_x2, arm_y2);

    // -----------------------------------------------
    // Sekcja: Rysowanie prostok¹ta reprezentuj¹cego cia³o quadrotora
    // -----------------------------------------------
    int rect_width = 70;
    int rect_height = 20;

    
    SDL_Point corners[4] = {
        {-rect_width / 2, -rect_height / 2},
        {rect_width / 2, -rect_height / 2},
        {rect_width / 2, rect_height / 2},
        {-rect_width / 2, rect_height / 2}
    };

    
    for (int i = 0; i < 4; i++) {
        float x = corners[i].x * cos(q_theta) - corners[i].y * sin(q_theta);
        float y = corners[i].x * sin(q_theta) + corners[i].y * cos(q_theta);
        
        corners[i].x = x + q_x;
        corners[i].y = y + q_y;
    }

    
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);

    Sint16 vx[4];
    Sint16 vy[4];

    for (int i = 0; i < 4; ++i) {
        vx[i] = corners[i].x;
        vy[i] = corners[i].y;
    }

    filledPolygonColor(gRenderer.get(), vx, vy, 4, 0xFF0000FF);

    // -----------------------------------------------
    // Sekcja: Rysowanie wirników quadrotora
    // -----------------------------------------------

    int propeller_radius = 10;

    // Animacja wirników - prze³¹czanie k¹ta co 100 ms
    Uint32 ticks = SDL_GetTicks();
    float propeller_angle = ((ticks / 100) % 2 == 0) ? 0 : M_PI; 

    float prop_arm_x1 = arm_x1 + propeller_radius * cos(propeller_angle + q_theta);
    float prop_arm_y1 = arm_y1 + propeller_radius * sin(propeller_angle + q_theta);

    float prop_arm_x3 = arm_x2 + propeller_radius * cos(propeller_angle + q_theta);
    float prop_arm_y3 = arm_y2 + propeller_radius * sin(propeller_angle + q_theta);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), prop_arm_x1, prop_arm_y1, arm_x1, arm_y1);
    SDL_RenderDrawLine(gRenderer.get(), prop_arm_x3, prop_arm_y3, arm_x2, arm_y2);
}