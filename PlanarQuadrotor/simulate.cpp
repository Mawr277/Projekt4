/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

float frequencyFactor = 1.0;
Mix_Chunk* sound = nullptr;

void frequencyEffect(int channel, void* stream, int len, void* udata) {

    float factor = *static_cast<float*>(udata);

    Sint16* samples = static_cast<Sint16*>(stream);

    int sample_count = len / sizeof(Sint16);
    std::vector<Sint16> new_samples;

    // Przetwarzanie próbek
    for (int i = 0; i < sample_count; ++i) {
        int index = static_cast<int>(i * factor);
        if (index < sample_count) {
            new_samples.push_back(samples[index]);
        }
    }

    // Kopiowanie przetworzonych próbek z powrotem do oryginalnego bufora
    for (int i = 0; i < std::min(sample_count, static_cast<int>(new_samples.size())); ++i) {
        samples[i] = new_samples[i];
    }
}


void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K, Mix_Chunk* sound) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    Eigen::Vector2f control_input = input - K * quadrotor.GetControlState();
    quadrotor.SetInput(control_input);

    frequencyFactor = std::min(2.0, 1.0 + control_input.norm() / 10.0);
    Mix_PlayChannel(-1, sound, 0);
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, Mix_Chunk*& sound, const int SCREEN_WIDTH, const int SCREEN_HEIGHT) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) >= 0) {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);

        if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
            std::cout << "SDL_mixer Error: " << Mix_GetError() << std::endl;
            return -1;
        }

        sound = Mix_LoadWAV("C:/Users/kryst/OneDrive/Desktop/STUDIA/quadr.wav");
        if (sound == nullptr) {
            std::cout << "Failed to load sound: " << Mix_GetError() << std::endl;
            return -1;
        }
    }
    else {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}

int main(int argc, char* args[]) {
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;

    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << 640, 360, 0, 0, 0, 0;
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 640, 360, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);

    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;

    if (init(gWindow, gRenderer, sound, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0) {
        Mix_RegisterEffect(MIX_CHANNEL_POST, frequencyEffect, nullptr, &frequencyFactor);

        SDL_Event e;
        bool quit = false;
        int x, y;

        while (!quit) {
            Eigen::VectorXf position = quadrotor.GetState();
            x_history.push_back(position[0]);
            y_history.push_back(position[1]);

            while (SDL_PollEvent(&e) != 0) {
                if (e.type == SDL_QUIT) {
                    quit = true;
                }
                if (e.type == SDL_KEYDOWN) {
                    
                    matplot::axis({ 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0 });
                    matplot::plot(x_history, y_history);
                }
                if (e.type == SDL_MOUSEBUTTONDOWN) {
                    SDL_GetMouseState(&x, &y);
                    goal_state << x, y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                    std::cout << "Set goal state to: (" << x << ", " << y << ")" << std::endl;
                }
                if (e.type == SDL_MOUSEMOTION) {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }
            }

            SDL_Delay(dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            control(quadrotor, K, sound);
            quadrotor.Update(dt);
        }

        Mix_UnregisterEffect(MIX_CHANNEL_POST, frequencyEffect);
    }

    Mix_FreeChunk(sound);
    Mix_CloseAudio();
    SDL_Quit();
    return 0;
}