#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/reset.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <fstream>
#include <algorithm>
#include <thread>
#include <sstream>

using namespace std;
using namespace std::chrono_literals;

#define MAZE_SIZE 29
#define GOAL_POS_R 14
#define GOAL_POS_C 14
#define UNKNOWN -1
#define FREE 0
#define WALL 1
#define GOAL 2

struct Pos {
    int r, c;
    Pos(int r=0, int c=0) : r(r), c(c) {}
    bool operator==(const Pos& o) const { return r==o.r && c==o.c; }
    bool operator!=(const Pos& o) const { return !(*this == o); }
};

map<string, pair<int,int>> DIR_DELTAS = {
    {"up", {-1,0}}, {"down", {1,0}}, {"left", {0,-1}}, {"right", {0,1}}
};

map<string, string> OPPOSITE = {
    {"up","down"}, {"down","up"}, {"left","right"}, {"right","left"}
};

// Removido sensores diagonais - apenas direções principais
map<string, pair<int,int>> SENSOR_OFFSETS = {
    {"up",{-1,0}}, {"down",{1,0}}, {"left",{0,-1}}, {"right",{0,1}}
};

vector<string> DIRS = {"up", "down", "left", "right"};

class BFSMapper : public rclcpp::Node {
public:
    BFSMapper() : Node("bfs_mapper"), pos(1,1), goal_pos(-1,-1), sensor_count(0) {
        // Inicializar mapa com posição inicial (1,1)
        for(int r=0; r<MAZE_SIZE; r++)
            for(int c=0; c<MAZE_SIZE; c++)
                maze[{r,c}] = UNKNOWN;
        maze[{1,1}] = FREE;
        
        // Clientes
        move_cli = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        reset_cli = create_client<cg_interfaces::srv::Reset>("/reset");
        
        RCLCPP_INFO(get_logger(), "Aguardando servicos...");
        move_cli->wait_for_service();
        reset_cli->wait_for_service();
        
        reset_game_same_map();
        
        // Subscriber
        sensors_sub = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            [this](const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
                latest_sensors = msg;
                sensor_count++;
            }
        );
        
        RCLCPP_INFO(get_logger(), "Aguardando sensores...");
        while(rclcpp::ok() && !latest_sensors) {
            rclcpp::spin_some(get_node_base_interface());
            this_thread::sleep_for(100ms);
        }
        
        if(!wait_until_moves_work(10.0)) {
            RCLCPP_ERROR(get_logger(), "Falha na sincronizacao!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "BFS MAPPER - Iniciando exploracao! Posicao inicial: (1,1)");
        
        // FASE 1: Exploração
        if(!explore_maze()) {
            RCLCPP_WARN(get_logger(), "Exploracao interrompida antes de chegar no goal");
            print_statistics();
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Exploracao concluida - GOAL ENCONTRADO!");
        print_statistics();
        
        // FASE 2: Calcular rota ótima
        auto learned_path = bfs_shortest_path(Pos(1,1), goal_pos);
        if(learned_path.empty()) {
            RCLCPP_ERROR(get_logger(), "Sem caminho ate goal");
            return;
        }
        
        auto learned_moves = path_to_moves(learned_path);
        RCLCPP_INFO(get_logger(), "Rota otima: %zu movimentos", learned_moves.size());
        
        compare_with_part1_log(learned_moves);
        
        // FASE 3: Executar rota ótima
        RCLCPP_INFO(get_logger(), "Resetando para executar rota...");
        reset_game_same_map();
        wait_until_moves_work(5.0);
        pos = Pos(1,1);
        
        RCLCPP_INFO(get_logger(), "Executando rota aprendida...");
        execute_moves(learned_moves);
        RCLCPP_INFO(get_logger(), "CHEGOU NO ALVO");
    }
    
private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_cli;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_cli;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensors_sub;
    
    Pos pos, goal_pos;
    map<pair<int,int>, int> maze;
    cg_interfaces::msg::RobotSensors::SharedPtr latest_sensors;
    int sensor_count;
    
    void reset_game_same_map() {
        auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
        req->is_random = false;
        req->map_name = "";
        auto fut = reset_cli->async_send_request(req);
        rclcpp::spin_until_future_complete(get_node_base_interface(), fut, 3s);
        
        for(int i=0; i<15; i++) {
            rclcpp::spin_some(get_node_base_interface());
            this_thread::sleep_for(50ms);
        }
    }
    
    bool wait_for_fresh_sensors(double timeout=2.0) {
        int start = sensor_count;
        auto start_time = chrono::steady_clock::now();
        
        while(rclcpp::ok() && sensor_count == start) {
            rclcpp::spin_some(get_node_base_interface());
            this_thread::sleep_for(10ms);
            
            auto elapsed = chrono::steady_clock::now() - start_time;
            if(chrono::duration<double>(elapsed).count() > timeout)
                return false;
        }
        
        for(int i=0; i<3; i++) {
            rclcpp::spin_some(get_node_base_interface());
            this_thread::sleep_for(10ms);
        }
        return true;
    }
    
    map<string,string> read_sensors() {
        map<string,string> s;
        if(!latest_sensors) return s;
        
        // Apenas sensores nas 4 direções principais
        s["up"] = latest_sensors->up;
        s["down"] = latest_sensors->down;
        s["left"] = latest_sensors->left;
        s["right"] = latest_sensors->right;
        
        return s;
    }
    
    void update_maze_from_sensors() {
        wait_for_fresh_sensors();
        auto sensors = read_sensors();
        
        maze[{pos.r, pos.c}] = FREE;
        
        // Processa apenas sensores nas 4 direções principais
        for(auto& [name, offset] : SENSOR_OFFSETS) {
            int nr = pos.r + offset.first;
            int nc = pos.c + offset.second;
            
            if(nr<0 || nr>=MAZE_SIZE || nc<0 || nc>=MAZE_SIZE) continue;
            
            string val = sensors[name];
            transform(val.begin(), val.end(), val.begin(), ::tolower);
            val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
            
            if(val == "b") {
                maze[{nr,nc}] = WALL;
            } else if(val == "t") {
                // Detectar goal apenas nas 4 direções adjacentes principais
                if(goal_pos == Pos(-1,-1)) {
                    maze[{nr,nc}] = GOAL;
                    goal_pos = Pos(nr,nc);
                    RCLCPP_INFO(get_logger(), "GOAL DETECTADO adjacente em (%d, %d)!", nr, nc);
                } else {
                    // Marcar como FREE para não bloquear
                    if(maze[{nr,nc}] == UNKNOWN) {
                        maze[{nr,nc}] = FREE;
                    }
                }
            } else if((val=="f" || val=="r") && maze[{nr,nc}]==UNKNOWN) {
                maze[{nr,nc}] = FREE;
            }
        }
    }
    
    bool move(string dir) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;
        auto fut = move_cli->async_send_request(req);
        
        if(rclcpp::spin_until_future_complete(get_node_base_interface(), fut, 2s)
           != rclcpp::FutureReturnCode::SUCCESS)
            return false;
        
        auto res = fut.get();
        if(res && res->success) {
            auto delta = DIR_DELTAS[dir];
            pos.r += delta.first;
            pos.c += delta.second;
            return true;
        }
        return false;
    }
    
    bool move_fast(string dir) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;
        auto fut = move_cli->async_send_request(req);
        
        if(rclcpp::spin_until_future_complete(get_node_base_interface(), fut, 1s)
           != rclcpp::FutureReturnCode::SUCCESS)
            return false;
        
        auto res = fut.get();
        if(res && res->success) {
            auto delta = DIR_DELTAS[dir];
            pos.r += delta.first;
            pos.c += delta.second;
            this_thread::sleep_for(120ms);
            rclcpp::spin_some(get_node_base_interface());
            return true;
        }
        return false;
    }
    
    bool wait_until_moves_work(double max_sec=10.0) {
        int attempts = static_cast<int>(max_sec / 0.2);
        for(int i=0; i<attempts; i++) {
            wait_for_fresh_sensors();
            auto sensors = read_sensors();
            
            for(auto& d : DIRS) {
                string val = sensors[d];
                transform(val.begin(), val.end(), val.begin(), ::tolower);
                val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
                
                if(val=="f" || val=="t") {
                    if(move(d)) {
                        move(OPPOSITE[d]);
                        return true;
                    }
                }
            }
            this_thread::sleep_for(100ms);
        }
        return false;
    }
    
    vector<pair<string,Pos>> get_accessible_neighbors(Pos p) {
        vector<pair<string,Pos>> neighbors;
        for(auto& [dir, delta] : DIR_DELTAS) {
            Pos n(p.r + delta.first, p.c + delta.second);
            if(n.r>=0 && n.r<MAZE_SIZE && n.c>=0 && n.c<MAZE_SIZE 
               && maze[{n.r,n.c}]!=WALL)
                neighbors.push_back({dir, n});
        }
        return neighbors;
    }
    
    vector<Pos> bfs_shortest_path(Pos start, Pos goal) {
        if(start == goal) return {start};
        
        queue<Pos> q;
        map<pair<int,int>, pair<int,int>> came_from;
        
        q.push(start);
        came_from[{start.r,start.c}] = {-1,-1};
        
        while(!q.empty()) {
            auto cur = q.front(); q.pop();
            if(cur == goal) break;
            
            for(auto& [_, delta] : DIR_DELTAS) {
                Pos n(cur.r + delta.first, cur.c + delta.second);
                if(!came_from.count({n.r,n.c}) && 
                   n.r>=0 && n.r<MAZE_SIZE && n.c>=0 && n.c<MAZE_SIZE &&
                   maze[{n.r,n.c}]!=WALL) {
                    came_from[{n.r,n.c}] = {cur.r, cur.c};
                    q.push(n);
                }
            }
        }
        
        if(!came_from.count({goal.r, goal.c})) return {};
        
        vector<Pos> path;
        auto cur = make_pair(goal.r, goal.c);
        while(cur.first != -1) {
            path.push_back(Pos(cur.first, cur.second));
            cur = came_from[cur];
        }
        reverse(path.begin(), path.end());
        return path;
    }
    
    vector<string> find_nearest_unknown() {
        queue<pair<Pos, vector<string>>> q;
        map<pair<int,int>, bool> visited;
        
        q.push({pos, {}});
        visited[{pos.r, pos.c}] = true;
        
        while(!q.empty()) {
            auto [cur, path] = q.front(); q.pop();
            
            for(auto& [dir, n] : get_accessible_neighbors(cur)) {
                if(visited.count({n.r, n.c})) continue;
                visited[{n.r, n.c}] = true;
                
                if(maze[{n.r, n.c}] == UNKNOWN) {
                    auto new_path = path;
                    new_path.push_back(dir);
                    return new_path;
                }
                
                auto new_path = path;
                new_path.push_back(dir);
                q.push({n, new_path});
            }
        }
        return {};
    }
    
    bool navigate_to_goal() {
        RCLCPP_INFO(get_logger(), "Navegando para o goal em (%d, %d)...", goal_pos.r, goal_pos.c);
        
        for(int step=0; step<200; step++) {
            if(pos == goal_pos) return true;
            
            update_maze_from_sensors();
            auto path = bfs_shortest_path(pos, goal_pos);
            
            if(path.empty() || path.size()<2) {
                RCLCPP_ERROR(get_logger(), "Sem caminho para o goal");
                return false;
            }
            
            Pos next = path[1];
            int dr = next.r - pos.r;
            int dc = next.c - pos.c;
            
            string dir;
            for(auto& [d, delta] : DIR_DELTAS)
                if(delta.first==dr && delta.second==dc) {
                    dir = d;
                    break;
                }
            
            if(step%10==0)
                RCLCPP_INFO(get_logger(), "  Navegando... %d passos, faltam %zu", step+1, path.size()-1);
            
            if(!move(dir))
                maze[{next.r, next.c}] = WALL;
        }
        return pos == goal_pos;
    }
    
    bool explore_maze() {
        RCLCPP_INFO(get_logger(), "Iniciando exploracao BFS...");
        
        for(int step=0; step<500; step++) {
            if(!rclcpp::ok()) break;
            
            update_maze_from_sensors();
            
            if(step%25==0) {
                int known=0;
                for(auto& [k,v] : maze) if(v!=UNKNOWN) known++;
                RCLCPP_INFO(get_logger(), "[%d] Pos: (%d, %d), Conhecidas: %d", step, pos.r, pos.c, known);
            }
            
            if(goal_pos != Pos(-1,-1)) {
                RCLCPP_INFO(get_logger(), "Goal encontrado, Navegando...");
                return navigate_to_goal();
            }
            
            if(pos == Pos(GOAL_POS_R, GOAL_POS_C)) {
                goal_pos = Pos(GOAL_POS_R, GOAL_POS_C);
                maze[{GOAL_POS_R, GOAL_POS_C}] = GOAL;
                RCLCPP_INFO(get_logger(), "Chegou no centro (goal)");
                return true;
            }
            
            auto neighbors = get_accessible_neighbors(pos);
            vector<pair<string,Pos>> unknown;
            for(auto& [d,n] : neighbors)
                if(maze[{n.r,n.c}]==UNKNOWN)
                    unknown.push_back({d,n});
            
            if(!unknown.empty()) {
                auto [dir, next] = unknown[0];
                if(move(dir)) continue;
                else {
                    maze[{next.r, next.c}] = WALL;
                    continue;
                }
            }
            
            auto path_to_unknown = find_nearest_unknown();
            if(path_to_unknown.empty()) {
                RCLCPP_INFO(get_logger(), "Explorou todas as celulas acessiveis");
                break;
            }
            
            for(auto& dir : path_to_unknown) {
                if(!move(dir)) {
                    auto delta = DIR_DELTAS[dir];
                    maze[{pos.r+delta.first, pos.c+delta.second}] = WALL;
                    break;
                }
                
                update_maze_from_sensors();
                if(goal_pos != Pos(-1,-1)) {
                    RCLCPP_INFO(get_logger(), "Goal encontrado durante exploracao!");
                    return navigate_to_goal();
                }
            }
        }
        
        RCLCPP_WARN(get_logger(), "Exploracao concluida sem encontrar o goal");
        return false;
    }
    
    vector<string> path_to_moves(vector<Pos>& path) {
        vector<string> moves;
        for(int i=1; i<path.size(); i++) {
            int dr = path[i].r - path[i-1].r;
            int dc = path[i].c - path[i-1].c;
            for(auto& [name, delta] : DIR_DELTAS)
                if(delta.first==dr && delta.second==dc) {
                    moves.push_back(name);
                    break;
                }
        }
        return moves;
    }
    
    void execute_moves(vector<string>& moves) {
        RCLCPP_INFO(get_logger(), "Executando em modo rapido...");
        for(int i=0; i<moves.size(); i++) {
            if(!move_fast(moves[i])) {
                RCLCPP_WARN(get_logger(), "Falha: %s (%d/%zu)", moves[i].c_str(), i+1, moves.size());
                break;
            }
            if((i+1)%15==0)
                RCLCPP_INFO(get_logger(), "Progresso: %d/%zu", i+1, moves.size());
        }
    }
    
    void print_statistics() {
        int known=0, free_c=0, walls=0;
        for(auto& [k,v] : maze) {
            if(v!=UNKNOWN) known++;
            if(v==FREE) free_c++;
            if(v==WALL) walls++;
        }
        
        RCLCPP_INFO(get_logger(), "=== ESTATISTICAS ===");
        RCLCPP_INFO(get_logger(), "Celulas conhecidas: %d", known);
        RCLCPP_INFO(get_logger(), "Celulas livres: %d", free_c);
        RCLCPP_INFO(get_logger(), "Paredes: %d", walls);
        if(goal_pos != Pos(-1,-1))
            RCLCPP_INFO(get_logger(), "Goal encontrado em: (%d, %d)", goal_pos.r, goal_pos.c);
        RCLCPP_INFO(get_logger(), "====================");
    }
    
    void compare_with_part1_log(vector<string>& learned) {
        string log_path = "C:\\dev\\cg_ws\\part1_route_log.txt";
        ifstream file(log_path);
        if(!file.is_open()) {
            RCLCPP_WARN(get_logger(), "Log da parte 1 nao encontrado em: %s", log_path.c_str());
            return;
        }
        
        string line;
        vector<string> part1;
        bool found = false;
        
        while(getline(file, line)) {
            if(line.find("Movimentos:") != string::npos) {
                found = true;
                if(getline(file, line)) {
                    line.erase(remove(line.begin(), line.end(), '['), line.end());
                    line.erase(remove(line.begin(), line.end(), ']'), line.end());
                    line.erase(remove(line.begin(), line.end(), '\''), line.end());
                    
                    stringstream ss(line);
                    string move;
                    while(getline(ss, move, ',')) {
                        move.erase(remove_if(move.begin(), move.end(), ::isspace), move.end());
                        if(!move.empty()) part1.push_back(move);
                    }
                }
                break;
            }
        }
        file.close();
        
        if(!found || part1.empty()) return;
        
        RCLCPP_INFO(get_logger(), "=== COMPARACAO COM PARTE 1 ===");
        RCLCPP_INFO(get_logger(), "Parte 1: %zu movimentos", part1.size());
        RCLCPP_INFO(get_logger(), "Parte 2: %zu movimentos", learned.size());
        
        if(learned == part1)
            RCLCPP_INFO(get_logger(), "ROTAS IDENTICAS");
        else if(learned.size() == part1.size())
            RCLCPP_INFO(get_logger(), "MESMO TAMANHO");
        else {
            int diff = abs((int)learned.size() - (int)part1.size());
            double pct = (double)diff / part1.size() * 100.0;
            RCLCPP_WARN(get_logger(), "Diferenca: %d movimentos (%.1f%%)", diff, pct);
        }
        RCLCPP_INFO(get_logger(), "=================================");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BFSMapper>();
    rclcpp::shutdown();
    return 0;
}