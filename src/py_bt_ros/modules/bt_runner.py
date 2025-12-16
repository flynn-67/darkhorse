import os
from modules.agent import Agent

class BTRunner:
    def __init__(self, config):
        self.config = config
        self.bt_viz_cfg = config['bt_runner'].get('bt_visualiser', {})
        self.bt_tick_rate = config['bt_runner']['bt_tick_rate']

        # 시각화 사용 여부
        self.viz_enabled = bool(self.bt_viz_cfg.get('enabled', False))

        # pygame 관련 멤버 기본값
        self.pygame = None
        self.screen = None
        self.clock = None
        self.bt_visualiser = None

        # ✅ enabled일 때만 pygame import/init
        if self.viz_enabled:
            import pygame
            self.pygame = pygame
            pygame.init()

            os.environ['SDL_VIDEO_WINDOW_POS'] = "0,30"
            self.screen_height = self.bt_viz_cfg.get('screen_height', 500)
            self.screen_width  = self.bt_viz_cfg.get('screen_width',  500)
            self.screen = pygame.display.set_mode(
                (self.screen_width, self.screen_height),
                pygame.RESIZABLE
            )

            from .bt_visualiser import BTViewer
            self.bt_visualiser = BTViewer(
                direction=self.bt_viz_cfg.get('direction', 'Vertical')
            )

            self.clock = pygame.time.Clock()

        # Initialise
        self.reset()

    def reset(self):
        self.running = True
        self.paused = False

        ros_namespace = self.config['agent'].get('namespaces', [])
        self.agent = Agent(ros_namespace)

        scenario_path = self.config['scenario'].replace('.', '/')
        behavior_tree_xml = (
            f"{os.path.dirname(os.path.dirname(os.path.abspath(__file__)))}/"
            f"{scenario_path}/{self.config['agent']['behavior_tree_xml']}"
        )
        self.agent.create_behavior_tree(str(behavior_tree_xml))

    async def step(self):
        await self.agent.run_tree()
        # ✅ pygame 없으면 tick도 그냥 스킵
        if self.viz_enabled and self.clock is not None:
            self.clock.tick(self.bt_tick_rate)

    def render(self):
        if not self.viz_enabled:
            return
        pygame = self.pygame
        self.bt_visualiser.render_tree(self.screen, self.agent.tree)

        if self.paused:
            font = pygame.font.Font(None, 48)
            text = font.render("Paused", True, (255, 0, 0))
            self.screen.blit(text, (10, 10))

        pygame.display.flip()

    def handle_keyboard_events(self):
        if not self.viz_enabled:
            return
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
