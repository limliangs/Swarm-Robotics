import pygame
import random
import math
import concurrent.futures

dt = 1

# Screen dimensions
width, height = 1200, 700

# Boid properties
num_boids = 10          # adjust number of boids
max_rank = 7            # default rank
min_speed = 3           # set minimum movement for swarming
max_speed = 4           # physical speed limit
max_force = 3           # maximum acceleration due to al, coh, sep
perception_radius = 200
safe_distance = 150     # distance which separation starts to be applied
danger_distance = 50

# Weights for each behaviour
weight_al = 6
weight_coh = 0.1
weight_sep = 0.4
weight_edge = 0
weight_target = 6

# Point of Interest properties
POI_radius = 30

# Colors
white = (255, 255, 255)
red = (255, 0, 0)
black = (0, 0, 0)
green = (0, 255, 0)
pink = (255, 100, 100)

class Boid:
# System Calls
    def __init__(self, label):
        self.label = label
        self.position = pygame.math.Vector2(random.uniform(500, 700), random.uniform(300, 400))    # randomize starting position
        self.velocity = pygame.math.Vector2(random.uniform(-1, 1), random.uniform(-1, 1))           # randomize velocity, scale to max_speed
        self.velocity.scale_to_length(max_speed)
        self.acceleration = pygame.math.Vector2(0, 0)                                               # initialize acceleration = 0
        self.angle = self.velocity.angle_to(pygame.math.Vector2(1, 0))                              # angle for visual aid when shown
        self.mode = 0                                                                               # 0 = swarming, 1 = approaching, 2 = arrive
        self.min_speed = min_speed
        self.targets = []
        self.ranks = []
        self.removed_targets = []
        self.current_targets_labelled = []
        self.new_targets_labelled = []
        self.timer = []
        self.continue_targets = []

    def apply_behavior(self, boids, POIs):          # Executed in "parallel" with other boids
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
        n_acceleration = weight_al * al + weight_coh * coh + weight_sep * 2 * sep + weight_edge * edge
        
        # Acceleration by target, weighted average
        selected = self.select_target_within()

        # If target exists within radius
        if selected != None:
            self.mode = 1
            tar = (self.follow_target(selected))
            n_acceleration += weight_target * tar
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge + weight_target)
        # If target exists outside radius, grouping
        elif self.targets != []:
            grouped_neighbours = self.grouping(neighbours)
            n_acceleration = pygame.math.Vector2(0, 0)  
            al = (self.align(grouped_neighbours))
            coh = (self.cohesion(grouped_neighbours))
            sep, weight_sep = self.separation(neighbours)
            edge = (self.avoid_edges())
            n_acceleration = weight_al * al + weight_coh * coh + weight_sep * 2 * sep + weight_edge * edge
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge)
        else:
            n_acceleration /= (weight_al + weight_coh + weight_sep + weight_edge)
        
        # Capping
        n_acceleration_cap, modi, possible = self.capping(n_acceleration)

        # Rank and targets
        new_targets, new_ranks, new_removed_targets, new_timer, continue_targets = self.update_POIs(POIs, neighbours)

        # Labels for printing
        self.current_targets_labelled = []
        for target in self.targets:
            self.current_targets_labelled.append(target.label)
        self.new_targets_labelled = []
        for target in new_targets:
            self.new_targets_labelled.append(target.label)

        # Debug message
        # print(f"{self.label}: type: {modi}, acc: {n_acceleration}, cap_acc: {n_acceleration_cap}, vel: {self.velocity}")
        # print(f"{self.label}: targets: {self.current_targets_labelled} -> {self.new_targets_labelled} ranks:   {self.ranks} -> {new_ranks}")
        
        # Return next acceleration and rank
        return new_targets, new_ranks, new_removed_targets, new_timer, continue_targets, n_acceleration_cap

    def update(self, new_targets, new_ranks, new_removed_targets, new_timer, continue_targets, n_acceleration):
        # update attributes from the previously computed values
        self.targets = new_targets
        self.ranks = new_ranks
        self.acceleration = n_acceleration
        self.removed_targets = new_removed_targets
        self.timer = new_timer
        self.continue_targets = continue_targets
        # self.acceleration = self.cap(n_acceleration)
        # freeze if arrived
        if self.mode == 2:      
            self.velocity = pygame.math.Vector2(0, 0)
            self.acceleration = pygame.math.Vector2(0, 0)
        # update angle, and bound & apply kinematics
        else:
            self.position += self.velocity * dt + 0.5 * self.acceleration * dt**2
            self.velocity += self.acceleration * dt

    def show(self, screen, font):
        pygame.draw.circle(screen, white, self.position, 3)
        
        # # Draw leader as green
        # if self.rank == 0:
        #     pygame.draw.circle(screen, green, self.position, 3)
        #     # pygame.draw.polygon(screen, green, [
        #     #     self.position + pygame.math.Vector2(math.cos(math.radians(self.angle)), -math.sin(math.radians(self.angle))) * 10,
        #     #     self.position + pygame.math.Vector2(math.cos(math.radians(self.angle + 160)), -math.sin(math.radians(self.angle + 160))) * 10,
        #     #     self.position + pygame.math.Vector2(math.cos(math.radians(self.angle - 160)), -math.sin(math.radians(self.angle - 160))) * 10,
        #     # ])
        # # Draw followers as white
        # else:
        #     pygame.draw.circle(screen, white, self.position, 3)
        #     # pygame.draw.polygon(screen, white, [
        #     #     self.position + pygame.math.Vector2(math.cos(math.radians(self.angle)), -math.sin(math.radians(self.angle))) * 10,
        #     #     self.position + pygame.math.Vector2(math.cos(math.radians(self.angle + 160)), -math.sin(math.radians(self.angle + 160))) * 10,
        #     #     self.position + pygame.math.Vector2(math.cos(math.radians(self.angle - 160)), -math.sin(math.radians(self.angle - 160))) * 10,
        #     # ])

        # Print rank on screen
        info_text1 = font.render(f"{self.label}", True, green)
        info_text2 = font.render(" ".join(f"[{item1}{item2}, {item3}]" for item1, item2, item3 in zip(self.current_targets_labelled, self.ranks, self.timer)), True, white)
        screen.blit(info_text2, (self.position.x + 15, self.position.y - 10))
        screen.blit(info_text1, (self.position.x + 15, self.position.y - 30))

# Functions
    def update_POIs(self, POIs, neighbours):
        new_targets = self.targets.copy()
        new_ranks = self.ranks.copy()
        new_timer = self.timer.copy()
        new_timer = [t - 1 for t in new_timer]
        continue_targets = []

        # Scan radius, claim leader
        for poi in POIs:
            if self.position.distance_to(poi.position) <= perception_radius:
                if poi not in new_targets:
                    new_targets.append(poi)
                    new_ranks.append(0)
                    new_timer.append(num_boids * 2)
        
        # If self is leader for target X
        for rank in new_ranks:
            if rank == 0:
                index = new_ranks.index(rank)
                target = new_targets[index]
                if all(target != i[0] for i in continue_targets):
                    continue_targets.append([target, num_boids]) # Append [target, countdown]
                    new_timer[index] = num_boids * 2

        # Add from neighbour's POIs, rank accordingly
        for neighbour in neighbours:
            for target in neighbour.targets:
                if target not in new_targets:
                    new_targets.append(target)
                    new_ranks.append(self.rank_from_neighbour(neighbour, target, max_rank))
                    # new_timer.append(self.lowest_time_neighbour(neighbour, target))\
                    new_timer.append(num_boids * 2)
                else:
                    index = new_targets.index(target)
                    current_rank = new_ranks[index]
                    new_ranks[index] = self.rank_from_neighbour(neighbour, target, current_rank)

        # Check to continue targets from neighbours ping
        for neighbour in neighbours:
            for target in neighbour.continue_targets:
                if target[1] > 0:
                    if all(target[0] != i[0] for i in continue_targets):
                        continue_targets.append([target[0], target[1] - 1])     # Append target to pass, decrement counter
                        if target[0] not in new_targets:
                            new_targets.append(target[0])
                            new_ranks.append(self.rank_from_neighbour(neighbour, target, max_rank))
                            new_timer.append(num_boids * 2)
                        else:
                            index = new_targets.index(target[0])
                            current_rank = new_ranks[index]
                            new_ranks[index] = self.rank_from_neighbour(neighbour, target, current_rank)
                            new_timer[index] = num_boids * 2
                else:
                    neighbour.continue_targets.remove(target)   #test debug, should be useless

        new_removed_targets = []
        # Check target completed within self radius to remove
        for target in self.targets:
            count = 0
            if self.position.distance_to(target.position) <= POI_radius:
                count += 1
                for neighbour in neighbours:
                    if neighbour.position.distance_to(target.position) <= POI_radius:
                        count += 1
                        if count == 3:
                            index = new_targets.index(target)
                            new_targets.remove(target)
                            new_removed_targets.append(target)   # remove from new_targets, append to removed_targets
                            new_ranks.pop(index)
                            new_timer.pop(index)
        
        # Check target completed outside radius using neighbours' removed targets
        for neighbour in neighbours:
            for target in neighbour.removed_targets:
                if target in new_targets:
                    index = new_targets.index(target)
                    new_targets.remove(target)
                    new_removed_targets.append(target)
                    new_ranks.pop(index)
                    new_timer.pop(index)

        # Check expired targets
        for t in new_timer:
            if t <= 0:      
                index = new_timer.index(t)
                new_timer.pop(index)
                try:
                    new_ranks.remove(new_ranks[index])
                    target = new_targets[index]
                    new_targets.remove(target)
                    new_removed_targets.append(target)
                except ValueError or IndexError:
                    print("FAILED TO REMOVE")
        return new_targets, new_ranks, new_removed_targets, new_timer, continue_targets

    def rank_from_neighbour(self, neighbour, target, current_rank):
        min_rank = current_rank
        try:
            index = neighbour.targets.index(target)     # Index of targets and ranks correspond to each other (hopefully)
            if neighbour.ranks[index] < min_rank:
                    min_rank = neighbour.ranks[index] + 1
        except ValueError:
            pass
        return min_rank

    def lowest_time_neighbour(self, neighbour, target):
        try:
            index_neigh = neighbour.targets.index(target)
            index_self = self.targets.index(target)
            if neighbour.timer[index_neigh] < self.timer[index_self] - 1:
                    min_timer = neighbour.timer[index_neigh]
                    return min_timer
        except ValueError:
            index_neigh = neighbour.targets.index(target)
            return neighbour.timer[index_neigh]
        return self.timer[index_self] - 1

    def check_leader(self, target, neighbours):                  # Search if leader for target is in network
        for neighbour in neighbours:
            if target in neighbour.targets:
                index = neighbour.targets.index(target)
                if neighbour.ranks[index] == 0:
                    return True
        return False

    def select_target_within(self):
        selected = None
        self.mode = 0
        closest = float('inf')
        for target in self.targets:
            distance = self.position.distance_to(target.position)
            if distance <= perception_radius:
                if distance < closest:
                    closest = distance
                    selected = target
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
            if boid != self and self.position.distance_to(boid.position) < perception_radius:
                neighbours.append(boid)
        return neighbours                   # returns a list of boids that are within perception_radius

    def grouping(self, neighbours):
        grouped_neighbours = []
        # Obtain target by lowest ranked
        try:
            index = self.ranks.index(min(self.ranks))
            target = self.targets[index]
            # Collect neighbours with same target
            for neighbour in neighbours:
                index = neighbour.ranks.index(min(neighbour.ranks))
                if neighbour.targets[index] == target:
                    grouped_neighbours.append(neighbour)
        except ValueError:
            pass
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
            weight_sep = (safe_distance - closest) / safe_distance
        return steering, weight_sep

    def follow_target(self, target):
        steering = pygame.math.Vector2(0, 0)
        if target != None:
            distance = self.position.distance_to(target.position)
            if distance <= POI_radius:
                self.mode = 2                                           # freeze
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
                    if self.count == 3:
                        POIs.remove(self)

def main():
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()

    boids = [Boid(chr(65 + i)) for i in range(num_boids)]
    POIs = []

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
        if mouse[0] == 1:
            x, y = pygame.mouse.get_pos()
            coordinate = pygame.math.Vector2(x, y)
            duplicate = False
            for poi in POIs:
                if poi.position == coordinate:
                    duplicate = True
            if duplicate == False:
                POIs.append(POI(x, y, chr(97 + i)))
                i += 1
        
        # Draw POI
        for poi in POIs:
            poi.update(boids, POIs, screen)
            poi.show(screen, font=pygame.font.Font(None, 24))

        # Use ThreadPoolExecutor to update each boid in parallel
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Apply behaviors and update boids concurrently
            futures = [executor.submit(boid.apply_behavior, boids, POIs) for boid in boids]
            concurrent.futures.wait(futures)  # Wait for all boids to complete calculation
            
            new_values = [future.result() for future in futures]
            for boid, (new_targets, new_ranks, new_removed_targets, new_timer, continue_targets, n_acceleration) in zip(boids, new_values):
                boid.update(new_targets, new_ranks, new_removed_targets, new_timer, continue_targets, n_acceleration)

        # Draw boids
        for boid in boids:
            boid.show_perception(boids, screen)
            boid.show(screen, font=pygame.font.Font(None, 24))

        print(f"Time Step: {ts}")
        ts += 1

        pygame.display.flip()
        # Adjust to see in slow motion. Try 15 or 10
        clock.tick(20)


    pygame.quit()

if __name__ == "__main__":
    main()