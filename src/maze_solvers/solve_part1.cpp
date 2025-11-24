#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <fstream>
#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace std::chrono_literals;

// Codigos internos do solver
#define FREE 0
#define WALL 1
#define ROBOT 2
#define GOAL 3

struct Pos {
    int r, c;
    bool operator==(const Pos& o) const { return r==o.r && c==o.c; }
};

map<string, pair<int,int>> DIRS = {
    {"up", {-1, 0}},
    {"down", {1, 0}},
    {"left", {0, -1}},
    {"right", {0, 1}}
};

class Part1Solver : public rclcpp::Node {
public:
    Part1Solver() : Node("part1_solver") {
        map_cli = create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_cli = create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        RCLCPP_INFO(get_logger(), "Aguardando /get_map e /move_command...");
        map_cli->wait_for_service();
        move_cli->wait_for_service();
        
        auto grid = fetch_map();
        auto [start, goal] = find_start_goal(grid);
        
        RCLCPP_INFO(get_logger(), "Start: (%d, %d), Goal: (%d, %d)", start.r, start.c, goal.r, goal.c);
        
        auto path = bfs(grid, start, goal);
        if(path.empty()) {
            RCLCPP_ERROR(get_logger(), "Nenhum caminho encontrado");
            return;
        }
        
        auto moves = path_to_moves(path);
        log_optimal_route(start, goal, path, moves);
        
        RCLCPP_INFO(get_logger(), "Movimentos (%zu): ", moves.size());
        
        execute_moves(moves);
        RCLCPP_INFO(get_logger(), "Chegou no alvo");
    }
    
private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_cli;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_cli;
    
    int to_cell(string s) {
        transform(s.begin(), s.end(), s.begin(), ::tolower);
        s.erase(remove_if(s.begin(), s.end(), ::isspace), s.end());
        
        if(s == "f") return FREE;
        if(s == "b") return WALL;
        if(s == "r") return ROBOT;
        if(s == "t") return GOAL;
        return FREE;
    }
    
    vector<vector<int>> fetch_map() {
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto fut = map_cli->async_send_request(req);
        rclcpp::spin_until_future_complete(get_node_base_interface(), fut);
        auto res = fut.get();
        
        auto& data_str = res->occupancy_grid_flattened;
        auto& shape = res->occupancy_grid_shape;
        
        int h = static_cast<int>(shape[0]);
        int w = static_cast<int>(shape[1]);
        
        RCLCPP_INFO(get_logger(), "Shape do mapa: %dx%d", h, w);
        
        vector<int> data;
        for(auto& s : data_str) data.push_back(to_cell(s));
        
        vector<vector<int>> grid(h, vector<int>(w));
        for(int i=0; i<h; i++)
            for(int j=0; j<w; j++)
                grid[i][j] = data[i*w + j];
        
        return grid;
    }
    
    pair<Pos, Pos> find_start_goal(vector<vector<int>>& grid) {
        Pos start{-1,-1}, goal{-1,-1};
        for(int r=0; r<grid.size(); r++)
            for(int c=0; c<grid[0].size(); c++) {
                if(grid[r][c] == ROBOT) start = {r, c};
                if(grid[r][c] == GOAL) goal = {r, c};
            }
        return {start, goal};
    }
    
    vector<Pos> bfs(vector<vector<int>>& grid, Pos start, Pos goal) {
        int h = grid.size(), w = grid[0].size();
        queue<Pos> q;
        map<pair<int,int>, pair<int,int>> prev;
        
        q.push(start);
        prev[{start.r, start.c}] = {-1, -1};
        
        while(!q.empty()) {
            auto cur = q.front(); q.pop();
            if(cur == goal) break;
            
            for(auto& [name, delta] : DIRS) {
                int nr = cur.r + delta.first;
                int nc = cur.c + delta.second;
                
                if(nr>=0 && nr<h && nc>=0 && nc<w && 
                   grid[nr][nc]!=WALL && !prev.count({nr,nc})) {
                    prev[{nr,nc}] = {cur.r, cur.c};
                    q.push({nr, nc});
                }
            }
        }
        
        if(!prev.count({goal.r, goal.c})) return {};
        
        vector<Pos> path;
        auto cur = make_pair(goal.r, goal.c);
        while(cur.first != -1) {
            path.push_back({cur.first, cur.second});
            cur = prev[cur];
        }
        reverse(path.begin(), path.end());
        return path;
    }
    
    vector<string> path_to_moves(vector<Pos>& path) {
        vector<string> moves;
        for(int i=1; i<path.size(); i++) {
            int dr = path[i].r - path[i-1].r;
            int dc = path[i].c - path[i-1].c;
            
            for(auto& [name, delta] : DIRS)
                if(delta.first==dr && delta.second==dc) {
                    moves.push_back(name);
                    break;
                }
        }
        return moves;
    }
    
    void execute_moves(vector<string>& moves) {
        for(auto& m : moves) {
            auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            req->direction = m;
            auto fut = move_cli->async_send_request(req);
            
            if(rclcpp::spin_until_future_complete(get_node_base_interface(), fut, 2s) 
               != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_WARN(get_logger(), "Falhou ao mover %s", m.c_str());
                break;
            }
        }
    }
    
    void log_optimal_route(Pos start, Pos goal, vector<Pos>& path, vector<string>& moves) {
        time_t now = time(0);
        tm* ltm = localtime(&now);
        
        stringstream ss;
        ss << "=== PART 1: ROTA PERFEITA ===" << endl;
        ss << "Timestamp: " << 1900+ltm->tm_year << "-" 
           << setfill('0') << setw(2) << 1+ltm->tm_mon << "-" 
           << setw(2) << ltm->tm_mday << " "
           << setw(2) << ltm->tm_hour << ":" 
           << setw(2) << ltm->tm_min << ":" 
           << setw(2) << ltm->tm_sec << endl;
        ss << "Start: (" << start.r << ", " << start.c << ")" << endl;
        ss << "Goal: (" << goal.r << ", " << goal.c << ")" << endl;
        ss << "Tamanho do caminho (celulas): " << path.size() << endl;
        ss << "Tamanho da rota (movimentos): " << moves.size() << endl << endl;
        
        ss << "Caminho (celulas):" << endl << "[";
        for(int i=0; i<path.size(); i++) {
            ss << "(" << path[i].r << ", " << path[i].c << ")";
            if(i < path.size()-1) ss << ", ";
        }
        ss << "]" << endl << endl;
        
        ss << "Movimentos:" << endl << "[";
        for(int i=0; i<moves.size(); i++) {
            ss << "'" << moves[i] << "'";
            if(i < moves.size()-1) ss << ", ";
        }
        ss << "]" << endl;
        ss << "==========================" << endl;
        
        string text = ss.str();
        RCLCPP_INFO(get_logger(), "\n%s", text.c_str());
        
        // Salvar em C:\dev\cg_ws (caminho fixo com permissão)
        string log_path = "C:\\dev\\cg_ws\\part1_route_log.txt";
        
        RCLCPP_INFO(get_logger(), "Tentando salvar log em: %s", log_path.c_str());
        ofstream file(log_path);
        if(file.is_open()) {
            file << text;
            file.close();
            RCLCPP_INFO(get_logger(), "Log salvo com sucesso!");
        } else {
            RCLCPP_ERROR(get_logger(), "ERRO: Nao conseguiu criar o arquivo!");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Part1Solver>();
    rclcpp::shutdown();
    return 0;
}