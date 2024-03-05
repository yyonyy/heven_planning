import matplotlib.pyplot as plt
import math

show_animation = True


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        # 시작 노드 초기화 x, y, cost, parent_index
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),self.calc_xy_index(sy, self.min_y), 0.0, -1)
        
        # 목표 노드 초기화 x, y, cost, parent_index
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        # open_set : 아직 방문하지 않은 노드, closed_set : 이미 방문한 노드
        open_set, closed_set = dict(), dict()
        
        # 시작 노드를 open_set에 추가
        open_set[self.calc_index(start_node)] = start_node

        #############################################################################
        #::::::::::::::::::::::::::::::main loop::::::::::::::::::::::::::::::::::::#
        #############################################################################
        
        while True:
            # open_set에 저장된 노드 중 cost가 가장 적은 노드의 index 찾기
            # 이 부분에 open_set의 cost에 휴리스틱 함수를 추가하여 astar를 구현해 보세요.
            c_id = min(open_set, key=lambda o: open_set[o].cost)

            # min_cost = float('inf')  # 초기값을 무한대로 설정
            # for node_id in open_set:
            #     current_cost = open_set[node_id].cost
            #     if current_cost < min_cost:
            #         min_cost = current_cost
            #         c_id = node_id
                        
            # 해당 노드 선택
            current = open_set[c_id]

            # show graph
            if show_animation:
                plt.plot(self.calc_position(current.x, self.min_x), self.calc_position(current.y, self.min_y), "xc")

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            ##########################################################################
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            ##########################################################################
            
            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,current.y + move_y, current.cost + move_cost, c_id)
                n_id = self.calc_index(node)
                
                # 이미 방문한 노드면 넘어감
                if n_id in closed_set:
                    continue
                
                # 갈 수 있는 노드인지 확인
                if not self.verify_node(node):
                    continue
                
                # 새로운 노드인 경우
                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node

                # open_set에 이미 존재하는 노드인 경우, 저장된 cost와 새로운 cost 비교
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        # 2D 그리드 상의 노드를 1차원 인덱스로 변환
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        # 클래스 멤버 변수인 obstacle_map 설정
        
        # 최소, 최대 x 및 y 위치 계산 및 출력
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        # x 및 y 너비 계산 및 출력
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # 순서대로 dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = -5.0  # [m]
    sy = -5.0  # [m]
    gx = 90.0  # [m]
    gy = 50.0  # [m]
    grid_size = 5.0  # [m]
    robot_radius = 2.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 100):
        ox.append(i)
        oy.append(-10.0)
        
    for i in range(-10,100):
        ox.append(i)
        oy.append(60.0)
        
    for i in range(-10, 60):
        ox.append(100.0)
        oy.append(i)

    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    
    # obstacles    
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
        
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    
    for i in range(-10, 40):
        ox.append(60.0)
        oy.append(i)
    
    for i in range(0, 40):
        ox.append(80.0)
        oy.append(60.0 - i)


    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
