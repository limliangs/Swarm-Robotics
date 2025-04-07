import pygame
import random
import math
import concurrent.futures

dt = 1
required_boids = 2

# Screen dimensions
width, height = 1200, 600

# Boid properties
num_boids = 5          # adjust number of boids
max_rank = 7            # default rank
min_speed = 3           # set minimum movement for swarming
max_speed = 4           # physical speed limit
max_force = 3           # maximum acceleration due to al, coh, sep
perception_radius = 200
safe_distance = 150     # distance which separation starts to be applied
safe_distance_wall = 50
danger_distance = 50
ttk = num_boids * 2

# Weights for each behaviour
weight_al = 10
weight_coh = 0.1
weight_sep = 0.8
weight_edge = 0
weight_target = 6

# Point of Interest properties
POI_radius = 30

spacing_line = radius = 1
spacing_nodes = 200

# Colors
white = (255, 255, 255)
red = (255, 0, 0)
black = (0, 0, 0)
green = (0, 255, 0)
blue = (50, 50, 255)
pink = (255, 100, 100)

class Boid:
# System Calls
    def __init__(self, label):
        self.label = label
        self.position = pygame.math.Vector2(random.uniform(550, 650), random.uniform(250, 350))
        # self.position = pygame.math.Vector2(random.uniform(25, 175), random.uniform(25, 175))       # randomize starting position
        self.velocity = pygame.math.Vector2(random.uniform(-1, 1), random.uniform(-1, 1))           # randomize velocity, scale to max_speed
        self.velocity.scale_to_length(max_speed)
        self.acceleration = pygame.math.Vector2(0, 0)                                               # initialize acceleration = 0
        self.angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))                              # angle for visual aid when shown
        self.mode = 0                                                                               # 0 = swarming, 1 = approaching, 2 = arrive
        self.min_speed = min_speed
        self.targets = []
        self.selected = None

    def apply_behavior(self, boids, POIs, walls):          # Executed in "parallel" with other boids
        neighbours = self.get_neighbours(boids)
        d_closest = self.get_closest_neighbour(neighbours)
        self.min_speed = self.set_min_speed(d_closest)

        # Acceleration by al, col, sep, edges
        n_acceleration = pygame.math.Vector2(0, 0)  
        al = (self.align(neighbours))
        coh = (self.cohesion(neighbours))
        sep, weight_sep = self.separation(neighbours)
        sep = (sep)
        edge = (self.avoid_edges())
        wall, weight_wall = (self.avoid_wall(walls))
        n_acceleration = weight_al * al + weight_coh * coh + weight_sep * sep + weight_edge * edge + wall * weight_wall

        # Rank and targets
        new_targets = self.update_POIs(POIs, neighbours)
        
        # Acceleration by selected
        # if selected exists
        if self.selected != None:
            # continue selected if no change in targets
            if self.targets == new_targets:
                selected = self.selected
            # recalculate selected if change in targets
            elif new_targets != []:
                min_value = min(target[1] for target in new_targets)
                indices = [i for i, target in enumerate(new_targets) if target[1] == min_value]
                selected_index = random.choice(indices)
                selected = new_targets[selected_index][0]
            else:
                selected = None
            
            if selected != None:
                if self.position.distance_to(selected.position) <= perception_radius:
                    
                    tar = (self.follow_target(selected))
                    if self.mode != 2:
                        self.mode = 1
                    n_acceleration += weight_target * tar
                    n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_target + weight_wall)
                else:
                    self.mode = 0
                    grouped_neighbours = self.grouping(neighbours)
                    n_acceleration = pygame.math.Vector2(0, 0)
                    al = (self.align(grouped_neighbours))
                    coh = (self.cohesion(grouped_neighbours))
                    sep, weight_sep = self.separation(neighbours)
                    edge = (self.avoid_edges())
                    n_acceleration = weight_al * al + weight_coh * coh + weight_sep * sep + weight_edge * edge + wall * weight_wall
                    n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_wall)
            else:
                n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_wall)
        # if selected doesnt exist yet
        else:
            selected = self.select_target_within()  # select target if any within radius            
            # If target exists within radius
            if selected != None and self.position.distance_to(selected.position) <= perception_radius:
                
                tar = (self.follow_target(selected))
                if self.mode != 2:
                    self.mode = 1
                n_acceleration += weight_target * tar
                n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_target + weight_wall)
            # If target exists outside radius, grouping
            elif new_targets != []:
                min_value = min(target[1] for target in new_targets)
                indices = [i for i, target in enumerate(new_targets) if target[1] == min_value]
                selected_index = random.choice(indices)
                selected = new_targets[selected_index][0]

                grouped_neighbours = self.grouping(neighbours)
                n_acceleration = pygame.math.Vector2(0, 0)
                al = (self.align(grouped_neighbours))
                coh = (self.cohesion(grouped_neighbours))
                sep, weight_sep = self.separation(neighbours)
                edge = (self.avoid_edges())
                n_acceleration = weight_al * al + weight_coh * coh + weight_sep * sep + weight_edge * edge + wall * weight_wall
                n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_wall)
            else:
                n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_wall)
        
        
        # Capping
        n_acceleration_cap, modi, possible = self.capping(n_acceleration)

        # Debug message
        # print(f"{self.label}: type: {modi}, acc: {n_acceleration}, cap_acc: {n_acceleration_cap}, vel: {self.velocity}")
        # print(f"{self.label}: targets: {self.current_targets_labelled} -> {self.new_targets_labelled} ranks:   {self.ranks} -> {new_ranks}")
        
        # Return next acceleration and rank
        return new_targets, n_acceleration_cap, selected
    
    def update(self, new_targets, n_acceleration, selected):
        # update attributes from the previously computed values
        self.targets = new_targets
        self.acceleration = n_acceleration
        if any(target[0] == selected for target in self.targets):
            self.selected = selected
            # print(f"{self.label}: selected")
        else:
            # print(f"{self.label}: selected not in self.targets")
            self.selected = None
        # self.acceleration = self.cap(n_acceleration)
        # freeze if arrived
        if self.mode == 2:      
            self.velocity = pygame.math.Vector2(0, 0)
            self.acceleration = pygame.math.Vector2(0, 0)
        # update angle, and bound & apply kinematics
        else:
            # self.position += self.velocity * dt + 0.5 * self.acceleration * dt**2
            self.velocity += self.acceleration * dt
            self.position += self.velocity * dt

    def show(self, screen, font):
        pygame.draw.circle(screen, white, self.position, 3)

        # self.current_targets_labelled = []
        # for target in self.targets:
        #     self.current_targets_labelled.append([target[0].label, target[1]])
        # # Print rank on screen
        # info_text1 = font.render(f"{self.label}", True, blue)
        # info_text2 = font.render(f"{self.current_targets_labelled}", True, white)

        # screen.blit(info_text1, (self.position.x, self.position.y - 30))
        # screen.blit(info_text2, (self.position.x, self.position.y - 10))

        # if self.selected != None:
        #     info_text3 = font.render(f"{self.selected.label}", True, green)
        #     screen.blit(info_text3, (self.position.x + 15, self.position.y - 30))

# Functions
    def update_POIs(self, POIs, neighbours):
        new_targets = []
        for target in self.targets:
            new_targets.append([target[0], num_boids])
        
        # Scan from neighbours
        for neighbour in neighbours:
            for target in neighbour.targets:
                # if target isnt registered, register then rank
                if all(target[0] != i[0] for i in new_targets):
                    first_rank = target[1] + 1
                    new_targets.append([target[0], first_rank])
                # if already, rank
                else:  
                    index = next(i for i, inner_list in enumerate(new_targets) if inner_list[0] == target[0])  # index on self list
                    new_targets[index][1] = min(new_targets[index][1], target[1] + 1)

        # Scan for poi in self.radius
        for poi in POIs:
            if self.position.distance_to(poi.position) <= perception_radius:
                if all(poi != i[0] for i in new_targets):
                    new_targets.append([poi, 0])
                else:
                    index = next(i for i, inner_list in enumerate(new_targets) if inner_list[0] == poi)  # index on self list
                    new_targets[index][1] = 0
                    
        # Scan for completion
        for target in new_targets:
            if target[1] >= num_boids:
                new_targets.remove([target[0], target[1]])
        
        return new_targets

    def rank_from_neighbour(self, neighbour, target, current_rank):
        min_rank = current_rank
        try:
            index = neighbour.targets.index(target)     # Index of targets and ranks correspond to each other (hopefully)
            if neighbour.ranks[index] < min_rank:
                    min_rank = neighbour.ranks[index] + 1
        except ValueError:
            pass
        return min_rank

    def select_target_within(self):
        selected = None
        self.mode = 0
        closest = float('inf')
        for target in self.targets:
            distance = self.position.distance_to(target[0].position)
            if distance <= perception_radius:
                if distance < closest:
                    closest = distance
                    selected = target[0]
        return selected

    def set_min_speed(self, closest):
        if closest <= danger_distance:
            return 0
        elif closest <= safe_distance:
            m = min_speed / (safe_distance - danger_distance)
            c = - m * danger_distance
            return m * closest + c
        else:
            return min_speed

    def get_closest_neighbour(self, neighbours):
        closest = float('inf')
        for neighbour in neighbours:
            distance = self.position.distance_to(neighbour.position)
            if distance < closest:
                closest = distance
        return closest

    def check_cond(self, centre, acc):              # Check if all 3 conditions are fulfiled
        eps = 1e-5
        if acc.length() <= max_force + eps and acc.distance_to(centre) <= max_speed/dt + eps and acc.distance_to(centre) >= self.min_speed/dt - eps:
            return True

    def capping(self, acc):
        capped_acc = None
        modi = None
        centre = -self.velocity/dt
        possible = []
        # Original
        possible.append([acc, "ORIGNL"])
        # Scaled
        if acc.length() > 0:
            scaled_acc = acc / acc.length() * max_force
            possible.append([scaled_acc, "SCALED"])
        #Projections
        if acc != centre:
            a, b = self.intersection_line_circle(centre=centre, radius=self.min_speed, point=acc)
            c, d = self.intersection_line_circle(centre=centre, radius=max_speed, point=acc)
            points = [a, b, c, d]
            for point in points:
                if point != None:
                    possible.append([point, "PROJEC"])
        # Intersections
        a, b = self.intersection_vertices(centre_a=pygame.math.Vector2(0,0), radius_a=max_force, centre_b=centre, radius_b=max_speed)
        c, d = self.intersection_vertices(centre_a=pygame.math.Vector2(0,0), radius_a=max_force, centre_b=centre, radius_b=self.min_speed)
        points = [a, b, c, d]
        for point in points:
            if point != None:
                possible.append([point, "INTSCT"])
        
        # Check all possibilities
        closest_distance = float('inf')
        for possibility in possible:
            distance = acc.distance_to(possibility[0])
            if distance < closest_distance and self.check_cond(centre, possibility[0]) == True:
                closest_distance = distance
                capped_acc = possibility[0]
                modi = possibility[1]

        return capped_acc, modi, possible

    def intersection_line_circle(self, centre, radius, point):
        direction = (point-centre).normalize()
        intersection_1 = centre + direction * radius
        intersection_2 = centre - direction * radius
        return intersection_1, intersection_2

    def intersection_vertices(self, centre_a, radius_a, centre_b, radius_b):
        d = centre_a.distance_to(centre_b)
        if d > radius_a + radius_b:             # Circles dont intersect
            return None, None
        elif d < abs(radius_a - radius_b):      # One circle is contained within the other
            return None, None
        elif d == 0 and radius_a == radius_b:   # The circles are identical and overlap completely.")
            return None, None
        
        # Point P where the line through the intersection points crosses the line between centers
        a = (radius_a**2 - radius_b**2 + d**2) / (2 * d)
        P = centre_a + a * (centre_b - centre_a).normalize()
        
        # Distance from point P to the intersection points
        h = math.sqrt(radius_a**2 - a**2)
        
        # Offset vector perpendicular to the line between centers
        offset = (centre_b - centre_a).normalize().rotate(90) * h
        
        # Intersection points in Vector2 format
        intersection_1 = P + offset
        intersection_2 = P - offset

        return intersection_1, intersection_2    

    def show_perception(self, neighbours, screen):  # Draws lines to neighbours
        for neighbour in neighbours:
            if self.position.distance_to(neighbour.position) < perception_radius:
                pygame.draw.line(screen, red, self.position, neighbour.position, 2)

    def get_neighbours(self, boids):
        neighbours = []
        for boid in boids:
            if boid != self and self.position.distance_to(boid.position) <= perception_radius:
                neighbours.append(boid)
        return neighbours                   # returns a list of boids that are within perception_radius

    def grouping(self, neighbours):
        grouped_neighbours = []
        
        for neighbour in neighbours:
            if neighbour.selected == self.selected:
                grouped_neighbours.append(neighbour)
        return grouped_neighbours


# Flocking Algorithms
    def align(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for neighbour in neighbours:
            steering += neighbour.velocity * 1                              # total velocity
            total += 1 * 1
            
            # if neighbour.rank < self.rank:
            #     steering += neighbour.velocity * 2                              # total velocity
            #     total += 1 * 2
            # elif neighbour.rank == self.rank:
            #     steering += neighbour.velocity * 1                              # total velocity
            #     total += 1 * 1
            # elif neighbour.rank > self.rank:
            #     steering += neighbour.velocity * 0.5                              # total velocity
            #     total += 1 * 0.5
        if total > 0:
            steering = steering/total                                   # avg velocity
            steering = (steering - self.velocity) / dt                  # acceleration needed to match avg velocity at next dt
        return steering

    def cohesion(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        for neighbour in neighbours:
            steering += neighbour.position * 1                              # total position
            total += 1 * 1
            
            # if neighbour.rank < self.rank:
            #     steering += neighbour.position * 2                              # total position
            #     total += 1 * 2
            # elif neighbour.rank == self.rank:
            #     steering += neighbour.position * 1                              # total position
            #     total += 1 * 1
            # elif neighbour.rank > self.rank:
            #     steering += neighbour.position * 0.5                              # total position
            #     total += 1 * 0.5
        if total > 0:
            steering /= total                                           # avg position
            target_pos = steering
            rel_target_pos = target_pos - self.position                                   # Delta p
            steering = 2 * (rel_target_pos - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        
        return steering

    def separation(self, neighbours):
        steering = pygame.math.Vector2(0, 0)
        total = 0
        closest = safe_distance + 1
        for neighbour in neighbours:
            distance = self.position.distance_to(neighbour.position)
            if distance < safe_distance:
                if distance < closest:
                    closest = distance
                away_v = -(neighbour.position - self.position)          # vector pointing away
                away_v.scale_to_length(safe_distance)                   # scaled vector to match safe_dist
                goal_p = neighbour.position + away_v                    # goal position w.r.t. one particular neighbour
                steering += goal_p                                      
                total += 1
        if total > 0:
            steering /= total                                           # avg goal position
            steering -= self.position                                   # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        if steering.length() == 0:
            weight_sep = 0
        else:
            weight_sep = (safe_distance - closest) / safe_distance      # closer closest, higher weight
        return steering, weight_sep

    def follow_target(self, target):
        steering = pygame.math.Vector2(0, 0)
        if target != None:
            distance = self.position.distance_to(target.position)
            if distance <= POI_radius:
                self.mode = 2                                           # freeze
            elif distance > POI_radius:
                self.mode = 0
            steering = target.position - self.position                  # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration required to achieve Deltap
        return steering

    def avoid_edges(self):
        steering = pygame.math.Vector2(0, 0)
        buffer = perception_radius  # Distance from edge to start avoiding
        
        if self.position.x < buffer:
            steering += pygame.math.Vector2(max_speed, 0)
            if self.position.x <= 0:
                self.velocity.x *= -1  # Bounce
        elif self.position.x > width - buffer:
            steering += pygame.math.Vector2(-max_speed, 0)
            if self.position.x >= width:
                self.velocity.x *= -1  # Bounce

        if self.position.y < buffer:
            steering += pygame.math.Vector2(0, max_speed)
            if self.position.y <= 0:
                self.velocity.y *= -1  # Bounce
        elif self.position.y > height - buffer:
            steering += pygame.math.Vector2(0, -max_speed)
            if self.position.y >= height:
                self.velocity.y *= -1  # Bounce

        if steering.length() > 0:
            steering.scale_to_length(max_speed)
            steering -= self.velocity
        return steering
    
    def avoid_wall(self, walls):
        # Add repulsion from wall particles
        steering = pygame.math.Vector2(0, 0)
        total = 0
        closest = safe_distance_wall + 1
        for wall_particle in walls:
            distance = self.position.distance_to(wall_particle.position)
            if distance < 5:
                self.velocity *= -1
            elif distance < safe_distance_wall:
                if distance < closest:
                    closest = distance
                away_v = -(wall_particle.position - self.position)          # vector pointing away
                away_v.scale_to_length(safe_distance_wall)                   # scaled vector to match safe_dist
                goal_p = wall_particle.position + away_v                    # goal position w.r.t. one particular neighbour
                steering += goal_p                                      
                total += 1
        if total > 0:
            steering /= total                                           # avg goal position
            steering -= self.position                                   # Delta p
            steering = 2 * (steering - self.velocity * dt) / dt**2      # acceleration needed to arrive at centre of mass at next dt
        if steering.length() == 0:
            weight_wall = 0
        else:
            weight_wall = (safe_distance_wall - closest) / safe_distance_wall      # closer closest, higher weight
        return steering, weight_wall * 16
        
# Additional
    def wraparound(self):
        if self.position.x < 0:
            self.position.x = (width + (self.position.x % width))
        if self.position.x >= width:
            self.position.x %= width
        if self.position.y < 0:
            self.position.y = (height + (self.position.y % height))
        if self.position.y >= height:
            self.position.y %= height

    def bounce(self):
        if self.position.x <= 0 or self.position.x >= width:
            self.velocity.x *= -1  # Bounce
        if self.position.y <= 0 or self.position.y >= height:
            self.velocity.x *= -1  # Bounce

class POI:
    def __init__(self, x, y, label):
        self.position = pygame.math.Vector2(x,y)
        self.count = 0
        self.label = label
    
    def show(self, screen, font):
        pygame.draw.circle(screen, pink, (self.position.x, self.position.y), POI_radius)
        pygame.draw.circle(screen, red, (self.position.x, self.position.y), 10)
        # Print rank on screen
        info_text = font.render(f"{self.label}", True, white)
        screen.blit(info_text, (self.position.x + 15, self.position.y - 10))

    def update(self, boids, POIs, screen):  # remove self if task is completed
        self.count = 0
        for boid in boids:
            distance = self.position.distance_to(boid.position)
            if distance < perception_radius:
                pygame.draw.line(screen, pink, self.position, boid.position, 2)
                if distance < POI_radius:
                    self.count += 1
                    if self.count == required_boids:
                        POIs.remove(self)
class WallParticle:
    def __init__(self, x, y, size):
        self.position = pygame.math.Vector2(x, y)  # Center position of the square
        self.size = size  # Side length of the square
        self.color = (255,0,0)  # Color for visualization

    def show(self, screen):
        # Calculate top-left corner from center position
        top_left = (self.position.x - self.size / 2, self.position.y - self.size / 2)
        rect = pygame.Rect(top_left[0], top_left[1], self.size, self.size)
        pygame.draw.rect(screen, self.color, rect)

def generate_grid(width, height, spacing_nodes):
    grid_points = []
    # Generate grid points
    for y in range(0, height, spacing_nodes):
        for x in range(0, width, spacing_nodes):
            grid_points.append([x,y])
    return grid_points

def randomize_bool(grid_points):
    map = []
    for point in grid_points:
        if point[0] == 0:       # left border
            map.append([random.choice([0,1]), 0])
        elif point[0] == width: # right border
            map.append([0,0])
        elif point[1] == 0:     # top border
            map.append([0, random.choice([0,1])])
        elif point[1] == height:  # bottom border
            map.append([0,0])
        else:
            map.append([random.choice([0,1]), random.choice([0,1])])
    return map
    
def create_wall_particles_line(x1, y1, x2, y2, spacing_line, radius=10):
    wall_particles = []
    
    # Calculate the direction vector and line length
    start = pygame.math.Vector2(x1, y1)
    end = pygame.math.Vector2(x2, y2)
    direction = (end - start).normalize()  # Unit vector pointing along the line
    line_length = start.distance_to(end)

    # Create particles along the line
    num_particles = int(line_length // spacing_line) + 1
    for i in range(num_particles):
        position = start + direction * (i * spacing_line)
        wall_particles.append(WallParticle(position.x, position.y, radius))

    return wall_particles

def create_walls_from_map(grid_points, wall_map, spacing_nodes, width, height):
    wall_particles = []
    
    # Loop through the grid and add walls based on boolean map
    for i in range(len(grid_points)):
        x, y = grid_points[i]
        x_next, y_next = grid_points[i+1] if i+1 < len(grid_points) else (None, None)  # Right neighbor
        x_down, y_down = grid_points[i+width//spacing_nodes] if i+width//spacing_nodes < len(grid_points) else (None, None)  # Bottom neighbor
        
        # Horizontal walls (between current point and right neighbor)
        if i % (width // spacing_nodes) != (width // spacing_nodes) - 1 and wall_map[i][0] == 1:  # If there is a wall in the x-direction
            if x_next is not None:
                wall_particles += create_wall_particles_line(x, y, x_next, y, spacing_line, radius)

        # Vertical walls (between current point and bottom neighbor)
        if wall_map[i][1] == 1:  # If there is a wall in the y-direction
            if y_down is not None:
                wall_particles += create_wall_particles_line(x, y, x, y_down, spacing_line, radius)
    
    return wall_particles

def main():
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()

    boids = [Boid(chr(65 + i)) for i in range(num_boids)]
    POIs = []
    walls = []
    walls += create_wall_particles_line(0,0, 0,height, spacing_line, radius) 
    walls += create_wall_particles_line(0,0, width,0, spacing_line, radius) 
    walls += create_wall_particles_line(width,height, width,0, spacing_line, radius)
    walls += create_wall_particles_line(width,height, 0, height, spacing_line, radius)
    

    # Generate grid points
    grid_points = generate_grid(width, height, spacing_nodes)

    # Generate the random map based on grid points
    random_map = randomize_bool(grid_points)
    walls += create_walls_from_map(grid_points, random_map, spacing_nodes, width, height)

    # walls = []
    # walls = [WallParticle(random.randint(0,width), random.randint(0,height), size = 10) for _ in range(50)]

    ts = 0  # Time step
    i = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(black)

        mouse = pygame.mouse.get_pressed()
        # Create POI at mouse click
        eps = 30
        if mouse[0] == 1:
            POIs = []
            # x, y = pygame.mouse.get_pos()
            # coordinate = pygame.math.Vector2(x, y)
            # duplicate = False
            # for poi in POIs:
            #     if (poi.position - coordinate).length() <= eps:
            #         duplicate = True
            # if duplicate == False:
            #     POIs.append(POI(x, y, chr(97 + i)))
            #     i += 1
            for _ in range(5):
                POIs.append(POI(random.randint(0,width), random.randint(0,height), chr(97 + i)))
                i += 1


        if mouse[2] == 0:
            fps = 20
        else:
            fps = 1
        
        # Draw walls
        for wall in walls:
            wall.show(screen)
        
        # Use ThreadPoolExecutor to update each boid in parallel
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Apply behaviors and update boids concurrently
            futures = [executor.submit(boid.apply_behavior, boids, POIs, walls) for boid in boids]
            concurrent.futures.wait(futures)  # Wait for all boids to complete calculation
            
            new_values = [future.result() for future in futures]
            for boid, (new_targets, n_acceleration, selected) in zip(boids, new_values):
                boid.update(new_targets, n_acceleration, selected)

        # Draw boids
        for boid in boids:
            # boid.show_perception(boids, screen)
            boid.show(screen, font=pygame.font.Font(None, 24))

        # Draw POI
        for poi in POIs:
            poi.update(boids, POIs, screen)
            poi.show(screen, font=pygame.font.Font(None, 24))


        print(f"Time Step: {ts}")
        ts += 1

        pygame.display.flip()
        # Adjust to see in slow motion. Try 15 or 10
        
        clock.tick(fps)


    pygame.quit()

if __name__ == "__main__":
    main()