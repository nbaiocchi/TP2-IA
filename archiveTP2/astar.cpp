/*
 Implementation of A* for solving planning problems
 Copyright (C) 2023 Christine Solnon
 Ce programme est un logiciel libre ; vous pouvez le redistribuer et/ou le modifier au titre des clauses de la Licence Publique Générale GNU, telle que publiée par la Free Software Foundation. Ce programme est distribué dans l'espoir qu'il sera utile, mais SANS AUCUNE GARANTIE ; sans même une garantie implicite de COMMERCIABILITE ou DE CONFORMITE A UNE UTILISATION PARTICULIERE. Voir la Licence Publique Générale GNU pour plus de détails.
 */

#include <queue>
#include <unordered_map>

using namespace std;

#include "state.cpp"
#include "stateGraph.cpp"

struct Info{
    int g;
    State pred;
    bool isGrey;
};

struct compare{
    // Used to compare elements in the priority queue
    bool operator()(pair<State,int> const& s1, pair<State,int> const& s2){
        return s1.second > s2.second;
    }
};

using PriorityQueue = priority_queue< pair<State,int>, vector<pair<State,int>>, compare >;
using HashMap = unordered_map<State, Info, State::hash>;

void printSolution(StateGraph &g, const State &s0, const State &s, const HashMap &map){
    // Print the sequence of actions to go from s0 to s
    if (!(s == s0)){
        printSolution(g, s0, map.at(s).pred, map);
        g.print(map.at(s).pred, s);
    }
}

void astar(const State &s0, StateGraph &g){
    // Search for the shortest path to go from s0 to a final state in g
    clock_t start = clock(); // start time used to compute CPU time
    int nbIter = 0;
    HashMap map;
    PriorityQueue q;
    map[s0].g = 0;
    map[s0].isGrey = true;
    q.push({s0, g.heuristic(s0)});
    while (!q.empty()){
        State s = q.top().first;
        q.pop();
        if (!map[s].isGrey) continue; // s is ignored because it is black
        nbIter++;
        if (g.isFinal(s)){
            printf("Optimal solution of length %d found in %d iterations and %f seconds\n", map.at(s).g, nbIter, ((double) (clock() - start)) / CLOCKS_PER_SEC);
            printSolution(g, s0, s, map);
            return;
        }
        int nbActions = g.searchActions(s);
        for (int i=0; i<nbActions; i++){
            State ss = g.transition(s, i);
            if (map.count(ss)==0 || map[s].g + g.getCost(s, i) < map[ss].g){
                map[ss].isGrey = true;
                map[ss].g = map[s].g + g.getCost(s, i);
                map[ss].pred = s;
                q.push( {ss, map[ss].g + g.heuristic(ss)} );
            }
        }
        map[s].isGrey = false;
    }
    printf("The problem has no solution (number of iterations = %d; CPU time = %fs)\n", nbIter, (double) (clock() - start) / CLOCKS_PER_SEC);
}

int main(){
    StateGraph g;
    astar(g.initialState(), g);
    return 0;
}




