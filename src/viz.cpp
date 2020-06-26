#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/vec3.hpp>   // glm::vec3
#include <glm/vec4.hpp>   // glm::vec4
#include <iostream>
#include <list>
#include <ostream>
#include <stdio.h>
#include <vector>

#include "imgui.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "imgui_plot.h"

#include <glad/glad.h>
// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

#include "agent.h"
#include "helpers.h"
#include "nodes.h"
#include "shaders.h"
#include "simulation.h"

#include "phi-messages/messages.pb.h"

#define U glm::dvec3(1.0, 0.0, 0.0)
#define V glm::dvec3(0.0, 1.0, 0.0)
#define W glm::dvec3(0.0, 0.0, 1.0)

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

glm::dvec3 cart_to_local(std::shared_ptr<Agent> Agent1,
                         std::shared_ptr<Agent> Agent2) {
  glm::dvec3 v = Agent2->get_position() - Agent1->get_position();
  return glm::inverse(Agent1->get_orientation()) * v;
}

glm::dvec3 angle(std::shared_ptr<Agent> Agent1, std::shared_ptr<Agent> Agent2) {
  glm::dvec3 dpos = cart_to_local(Agent1, Agent2);
  double rho = glm::length(dpos);
  double theta = glm::acos(dpos.z / rho);
  double phi = glm::atan(dpos.y, dpos.x);
  return glm::dvec3(rho, theta, phi);
}

class Drone : public Agent {
  void handleMessage(int32_t meso_type, std::string meso_content) {}
  void main() {}
};

class DroneSimulation : public Simulation {
public:
  DroneSimulation() {
    map_type map;
    map["Drone"] = &createInstance<Drone>;
    map["Client"] = &createInstance<Drone>;
    this->set_map(map);
  }

  void init() {}
  void environment() {}
  void handleMesoCustom(int32_t meso_type, std::string meso_content) {}

  void parse_values(std::string bytes) {}

  void parse_agent_infos(std::string bytes) {
    phi::AgentInfos agents = phi::AgentInfos();
    phi::Integers integers = phi::Integers();
    phi::Floats floats = phi::Floats();
    phi::Doubles doubles = phi::Doubles();
    phi::Strings strings = phi::Strings();
    agents.ParseFromString(bytes);

    int i = 0;
    for (auto agent_info : agents.infos()) {
      if (i >= this->get_number_of_agents()) {
        std::cout << "New agent type: \n" << std::flush;
        std::cout << agent_info.agent_type() << "\n" << std::flush;
        std::shared_ptr<Agent> agent = this->new_agent(agent_info.agent_type());
        std::cout << "Hello\n" << std::flush;
        this->add_agent(agent);
        this->set_number_of_agents(this->get_number_of_agents() + 1);
      }
      this->agents[i]->set_id(i);
      this->agents[i]->set_type(agent_info.agent_type());
      this->agents[i]->set_position({agent_info.agent_position(0),
                                     agent_info.agent_position(1),
                                     agent_info.agent_position(2)});
      this->agents[i]->set_orientation(
          {agent_info.agent_orientation(0), agent_info.agent_orientation(1),
           agent_info.agent_orientation(2), agent_info.agent_orientation(3)});
      for (phi::Value val : agent_info.values()) {
        std::cout << "Value '" << val.name() << "' received.\n";
        this->agents[i]->generic_store[val.name()] =
            std::make_unique<VectorStore>(val.name());
        if (val.has_integers()) {
          this->agents[i]->generic_store[val.name()]->set(std::vector<int>(
              val.integers().values().begin(), val.integers().values().end()));
        } else if (val.has_floats()) {
          this->agents[i]->generic_store[val.name()]->set(std::vector<float>(
              val.floats().values().begin(), val.floats().values().end()));
        } else if (val.has_doubles()) {
          this->agents[i]->generic_store[val.name()]->set(std::vector<double>(
              val.doubles().values().begin(), val.doubles().values().end()));
        } else if (val.has_strings()) {
          this->agents[i]->generic_store[val.name()]->set(
              std::vector<std::string>(val.strings().values().begin(),
                                       val.strings().values().end()));
        } else {
          std::cout << "Error: no value.\n";
        }
        // this->agents[i]->generic_store[val.name()]->print();
      }

      for (phi::Map map : agent_info.maps()) {
        std::cout << "Map '" << map.name() << "' received.\n";
        this->agents[i]->generic_map_store[map.name()] =
            std::make_unique<MapStore>(map.name());
        for (phi::MapValue map_value : map.values()) {
          if (map_value.key_case() == 1) { /* Int */
            if (map_value.has_integers()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.int_key(),
                  std::vector<int>(map_value.integers().values().begin(),
                                   map_value.integers().values().end()));
            } else if (map_value.has_floats()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.int_key(),
                  std::vector<float>(map_value.floats().values().begin(),
                                     map_value.floats().values().end()));
            } else if (map_value.has_doubles()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.int_key(),
                  std::vector<double>(map_value.doubles().values().begin(),
                                      map_value.doubles().values().end()));
            } else if (map_value.has_strings()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.int_key(),
                  std::vector<std::string>(map_value.strings().values().begin(),
                                           map_value.strings().values().end()));
            } else {
              std::cout << "Map has no data!\n";
            }
          } else if (map_value.key_case() == 2) { /* Float */
            if (map_value.has_integers()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.float_key(),
                  std::vector<int>(map_value.integers().values().begin(),
                                   map_value.integers().values().end()));
            } else if (map_value.has_floats()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.float_key(),
                  std::vector<float>(map_value.floats().values().begin(),
                                     map_value.floats().values().end()));
            } else if (map_value.has_doubles()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.float_key(),
                  std::vector<double>(map_value.doubles().values().begin(),
                                      map_value.doubles().values().end()));
            } else if (map_value.has_strings()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.float_key(),
                  std::vector<std::string>(map_value.strings().values().begin(),
                                           map_value.strings().values().end()));
            } else {
              std::cout << "Map has no data!\n";
            }
          } else if (map_value.key_case() == 3) { /* Double */
            if (map_value.has_integers()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.double_key(),
                  std::vector<int>(map_value.integers().values().begin(),
                                   map_value.integers().values().end()));
            } else if (map_value.has_floats()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.double_key(),
                  std::vector<float>(map_value.floats().values().begin(),
                                     map_value.floats().values().end()));
            } else if (map_value.has_doubles()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.double_key(),
                  std::vector<double>(map_value.doubles().values().begin(),
                                      map_value.doubles().values().end()));
            } else if (map_value.has_strings()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.double_key(),
                  std::vector<std::string>(map_value.strings().values().begin(),
                                           map_value.strings().values().end()));
            } else {
              std::cout << "Map has no data!\n";
            }
          } else if (map_value.key_case() == 4) { /* String */
            if (map_value.has_integers()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.string_key(),
                  std::vector<int>(map_value.integers().values().begin(),
                                   map_value.integers().values().end()));
            } else if (map_value.has_floats()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.string_key(),
                  std::vector<float>(map_value.floats().values().begin(),
                                     map_value.floats().values().end()));
            } else if (map_value.has_doubles()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.string_key(),
                  std::vector<double>(map_value.doubles().values().begin(),
                                      map_value.doubles().values().end()));
            } else if (map_value.has_strings()) {
              this->agents[i]->generic_map_store[map.name()]->set(
                  map_value.string_key(),
                  std::vector<std::string>(map_value.strings().values().begin(),
                                           map_value.strings().values().end()));
            } else {
              std::cout << "Map has no data!\n";
            }
          } else {
            std::cout << "Map has no data!\n";
          }
        }
        // this->agents[i]->generic_map_store[map.name()]->print();
      }
      ++i;
    }
  }

  std::vector<float> get_position_buffer() {
    std::vector<float> buffer;
    buffer.resize(3 * 2 * 4 * this->get_number_of_agents());

    int i = 0;
    for (auto agent : this->agents) {
      // std::cout << "Agent " << i << ", " << agent.type << ", " <<
      // glm::to_string(agent->get_position()) << ", " <<
      // glm::to_string(agent.orientation) << "\n";
      glm::dvec4 rU = agent->get_orientation() * glm::dvec4(U, 0.0);
      glm::dvec4 rV = agent->get_orientation() * glm::dvec4(V, 0.0);
      glm::dvec4 rW = agent->get_orientation() * glm::dvec4(W, 0.0);
      int start = 3 * 2 * 4 * i;

      buffer[start + 8 * 0 + 0] = agent->get_position().x;
      buffer[start + 8 * 0 + 1] = agent->get_position().y;
      buffer[start + 8 * 0 + 2] = agent->get_position().z;
      buffer[start + 8 * 0 + 3] = 1.0;

      buffer[start + 8 * 0 + 4] = agent->get_position().x + rU.x;
      buffer[start + 8 * 0 + 5] = agent->get_position().y + rU.y;
      buffer[start + 8 * 0 + 6] = agent->get_position().z + rU.z;
      buffer[start + 8 * 0 + 7] = 1.0;

      buffer[start + 8 * 1 + 0] = agent->get_position().x;
      buffer[start + 8 * 1 + 1] = agent->get_position().y;
      buffer[start + 8 * 1 + 2] = agent->get_position().z;
      buffer[start + 8 * 1 + 3] = 1.0;

      buffer[start + 8 * 1 + 4] = agent->get_position().x + rV.x;
      buffer[start + 8 * 1 + 5] = agent->get_position().y + rV.y;
      buffer[start + 8 * 1 + 6] = agent->get_position().z + rV.z;
      buffer[start + 8 * 1 + 7] = 1.0;

      buffer[start + 8 * 2 + 0] = agent->get_position().x;
      buffer[start + 8 * 2 + 1] = agent->get_position().y;
      buffer[start + 8 * 2 + 2] = agent->get_position().z;
      buffer[start + 8 * 2 + 3] = 1.0;

      buffer[start + 8 * 2 + 4] = agent->get_position().x + rW.x;
      buffer[start + 8 * 2 + 5] = agent->get_position().y + rW.y;
      buffer[start + 8 * 2 + 6] = agent->get_position().z + rW.z;
      buffer[start + 8 * 2 + 7] = 1.0;
      ++i;
    }

    return buffer;
  }
};

int main(int, char **) {
  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit())
    return 1;
  // GL 3.0 + GLSL 130
  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
  // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+
  // only
  // Create window with graphics context
  GLFWwindow *window = glfwCreateWindow(
      1280, 720, "Dear ImGui GLFW+OpenGL3 example", NULL, NULL);
  if (window == NULL)
    return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  bool err = gladLoadGL() == 0;

  if (err) {
    fprintf(stderr, "Failed to initialize OpenGL loader!\n");
    return 1;
  }
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  // Setup Dear ImGui style();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  /* Framebuffer for OpenGL */
  int width = 1000;
  int height = 1000;

  /* We create the framebuffer and bind to it */
  unsigned int framebuffer;
  glGenFramebuffers(1, &framebuffer);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
  glViewport(0, 0, width, height);

  // We generate a texture image and we attach it as a color attachement to the
  // framebuffer
  unsigned int texColorBuffer;
  glGenTextures(1, &texColorBuffer);
  glBindTexture(GL_TEXTURE_2D, texColorBuffer);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  // attach it to currently bound framebuffer object
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         texColorBuffer, 0);

  // We generate a renderbuffer object that will be used to enable depth testing
  // (and stencil testing)
  unsigned int rbo;
  glGenRenderbuffers(1, &rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // Attach renderbuffer object to the depth and stencil attachement of the
  // framebuffer
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER, rbo);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!"
              << std::endl;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // End of block « Framebuffer for OpenGL »

  //
  // GRID SHADER
  //

  int success;
  char infoLog[512];

  int gridVertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(gridVertexShader, 1, &gridVertexShaderSource, NULL);
  glCompileShader(gridVertexShader);
  glGetShaderiv(gridVertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(gridVertexShader, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  int gridFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(gridFragmentShader, 1, &gridFragmentShaderSource, NULL);
  glCompileShader(gridFragmentShader);
  // check for shader compile errors
  glGetShaderiv(gridFragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(gridFragmentShader, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  int gridShaderProgram = glCreateProgram();
  glAttachShader(gridShaderProgram, gridVertexShader);
  glAttachShader(gridShaderProgram, gridFragmentShader);
  glLinkProgram(gridShaderProgram);
  // check for linking errors
  glGetProgramiv(gridShaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(gridShaderProgram, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
  }
  glDeleteShader(gridVertexShader);
  glDeleteShader(gridFragmentShader);

  //
  // NODES SHADER
  //

  int nodesVertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(nodesVertexShader, 1, &nodesVertexShaderSource, NULL);
  glCompileShader(nodesVertexShader);
  glGetShaderiv(nodesVertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(nodesVertexShader, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  int nodesFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(nodesFragmentShader, 1, &nodesFragmentShaderSource, NULL);
  glCompileShader(nodesFragmentShader);
  // check for shader compile errors
  glGetShaderiv(nodesFragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(nodesFragmentShader, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }

  int nodesShaderProgram = glCreateProgram();
  glAttachShader(nodesShaderProgram, nodesVertexShader);
  glAttachShader(nodesShaderProgram, nodesFragmentShader);
  glLinkProgram(nodesShaderProgram);
  // check for linking errors
  glGetProgramiv(nodesShaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(nodesShaderProgram, 512, NULL, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
              << infoLog << std::endl;
  }
  glDeleteShader(nodesVertexShader);
  glDeleteShader(nodesFragmentShader);

  //
  // GRID DATA
  //
  float gridSize = 10.0;
  float gridSubdivision = 10.0;
  std::vector<float> gridVertices;

  for (float x = -0.01; x <= gridSize; x += (gridSize / gridSubdivision)) {
    gridVertices.push_back(x);
    gridVertices.push_back(0.0);
    gridVertices.push_back(0.0);
    gridVertices.push_back(1.0);
    gridVertices.push_back(x);
    gridVertices.push_back(gridSize);
    gridVertices.push_back(0.0);
    gridVertices.push_back(1.0);
  }

  for (float y = -0.01; y <= gridSize; y += (gridSize / gridSubdivision)) {
    gridVertices.push_back(0.0);
    gridVertices.push_back(y);
    gridVertices.push_back(0.0);
    gridVertices.push_back(1.0);
    gridVertices.push_back(gridSize);
    gridVertices.push_back(y);
    gridVertices.push_back(0.0);
    gridVertices.push_back(1.0);
  }

  unsigned int gridVBO, gridVAO, gridAttr;
  glGenVertexArrays(1, &gridVAO);
  glGenBuffers(1, &gridVBO);
  glBindVertexArray(gridVAO);

  glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * gridVertices.size(),
               &gridVertices.front(), GL_DYNAMIC_DRAW);
  gridAttr = glGetAttribLocation(gridShaderProgram, "posAttr");
  glVertexAttribPointer(gridAttr, 4, GL_FLOAT, GL_FALSE, 0, (void *)0);
  // glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  //
  // NODES DATA
  //
  std::vector<float> nodes;

  unsigned int nodesVBO, nodesVAO, nodesAttr;
  glGenVertexArrays(1, &nodesVAO);
  glGenBuffers(1, &nodesVBO);
  glBindVertexArray(nodesVAO);

  glBindBuffer(GL_ARRAY_BUFFER, nodesVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * nodes.size(), &nodes.front(),
               GL_DYNAMIC_DRAW);
  nodesAttr = glGetAttribLocation(nodesShaderProgram, "posAttr");
  glVertexAttribPointer(nodesAttr, 4, GL_FLOAT, GL_FALSE, 0, (void *)0);
  glEnableVertexAttribArray(nodesAttr);
  // glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  //
  // POWER GRAPHS
  //
  std::map<int32_t, DroneSimulation> simulations;
  int current_simulation = -1;
  bool plot = false;

  //
  // ZMQ
  //
  zmq::context_t *zmq_context = new zmq::context_t();
  zmq::socket_t *zmq_socket =
      new zmq::socket_t(*zmq_context, zmq::socket_type::sub);
  zmq_socket->bind("ipc:///tmp/1");
  zmq::pollitem_t items[] = {
      {static_cast<void *>(*zmq_socket), 0, ZMQ_POLLIN, 0}};
  zmq_socket->setsockopt(ZMQ_SUBSCRIBE, "Viz", 1); /* Là */

  //
  // VISUALIZATION PARAMETERS
  //

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  float zoom = 0.06;

  /* Matrix to flip the texture vertically */
  glm::mat4 flip = glm::mat4(1.0);
  flip[1][1] = -1.0;

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
    // tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
    // your main application.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
    // data to your main application. Generally you may always pass all inputs
    // to dear imgui, and hide them from your application based on those two
    // flags.
    glfwPollEvents();

    //
    // We check if we have updates to display
    //
    zmq::poll(&items[0], 1, 0);
    if (items[0].revents & ZMQ_POLLIN) {
      std::string address = s_recv(*zmq_socket); /* Là */
      std::string data = s_recv(*zmq_socket);
      phi::Viz viz = phi::Viz();
      viz.ParseFromString(data);

      if (simulations.find(viz.simulation_id()) == simulations.end()) {
        std::cout << "New simulation: " << viz.simulation_id() << "\n";
        current_simulation = viz.simulation_id();

        DroneSimulation simulation = DroneSimulation();
        simulation.set_id(viz.simulation_id());
        simulation.set_cursor(viz.clock());
        simulations[simulation.get_id()] = simulation;
      }

      switch (viz.type()) {
      case phi::Viz_MessageType_POSITIONS:
        simulations[viz.simulation_id()].parse_agent_infos(viz.content());
        break;
      case phi::Viz_MessageType_THROUGHPUTS:
        // simulations[viz.simulation_id()].parse_agent_throughputs(viz.content(),
        // (float) viz.clock());
        break;
      default:
        std::cout << "Default message. Wtf?\n";
      }
    }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin(
        "Display Configuration"); // Pass a pointer to our bool variable (the
                                  // window will have a closing button that will
                                  // clear the bool when clicked)
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::SliderFloat("Zoom", &zoom, 0.02, 1.0);
    ImGui::InputInt("Simulation ID", &current_simulation);
    ImGui::End();

    if (simulations.find(current_simulation) != simulations.end()) {
      nodes = simulations[current_simulation].get_position_buffer();
      plot = true;
    } else {
      nodes.clear();
      plot = false;
    }

    if (plot) {
      for (auto agent : simulations[current_simulation].agents) {
        if (agent->generic_map_store.find("power_curves") !=
            agent->generic_map_store.end()) {
          ImGui::Begin(
              (agent->get_type() + " " + std::to_string(agent->get_id()))
                  .c_str());

          ImVec2 wMin = ImGui::GetWindowContentRegionMin();
          ImVec2 wMax = ImGui::GetWindowContentRegionMax();
          wMin.x += ImGui::GetWindowPos().x;
          wMin.y += ImGui::GetWindowPos().y;
          wMax.x += ImGui::GetWindowPos().x;
          wMax.y += ImGui::GetWindowPos().y;

          if (agent->generic_store.find("goal") != agent->generic_store.end()) {
            double goal_cursor =
                std::get<int>(agent->generic_store["goal"]->get(0)) *
                (wMax.x - wMin.x) / 360.0f;
            ImGui::GetWindowDrawList()->AddLine(
                ImVec2(wMin.x + goal_cursor, wMin.y),
                ImVec2(wMin.x + goal_cursor, wMax.y),
                ImGui::GetColorU32(ImVec4(1.0, 0.0, 0.0, 1.0)), 6.0f);
          }

          if (agent->generic_store.find("angle") !=
              agent->generic_store.end()) {
            double angle_cursor =
                std::get<int>(agent->generic_store["angle"]->get(0)) *
                (wMax.x - wMin.x) / 360.0f;
            ImGui::GetWindowDrawList()->AddLine(
                ImVec2(wMin.x + angle_cursor, wMin.y),
                ImVec2(wMin.x + angle_cursor, wMax.y),
                ImGui::GetColorU32(ImVec4(0.0, 1.0, 0.0, 1.0)), 2.0f);
          }

          if (agent->generic_store.find("range") !=
              agent->generic_store.end()) {
            int angle = std::get<int>(agent->generic_store["angle"]->get(0));
            double vmin = constrainAngleDeg(
                angle -
                (std::get<int>(agent->generic_store["range"]->get(0)) / 2));
            double vmax = constrainAngleDeg(
                angle +
                (std::get<int>(agent->generic_store["range"]->get(0)) / 2));
            double mincursor = vmin * (wMax.x - wMin.x) / 360.0f;
            double maxcursor = vmax * (wMax.x - wMin.x) / 360.0f;
            std::cout << vmin << ", " << vmax << "\n";
            ImGui::GetWindowDrawList()->AddLine(
                ImVec2(wMin.x + mincursor, wMin.y),
                ImVec2(wMin.x + mincursor, wMax.y),
                ImGui::GetColorU32(ImVec4(1.0, 0.0, 0.0, 1.0)), 6.0f);
            ImGui::GetWindowDrawList()->AddLine(
                ImVec2(wMin.x + maxcursor, wMin.y),
                ImVec2(wMin.x + maxcursor, wMax.y),
                ImGui::GetColorU32(ImVec4(0.0, 0.0, 1.0, 1.0)), 2.0f);
          }

          std::vector<std::vector<float>> data(
              agent->generic_map_store["power_curves"]->size());
          const float *y_data[agent->generic_map_store["power_curves"]->size()];
          int i = 0;
          for (const auto &[key, arr] :
               *(agent->generic_map_store["power_curves"])) {
            for (auto v : std::get<std::vector<double>>(arr))
              data[i].push_back(v);
            y_data[i] = data[i].data();
            ++i;
          }

          uint32_t an = 100;
          ImGui::PlotConfig conf;
          conf.values.count = 360;
          conf.values.ys_list = y_data;
          conf.values.ys_count =
              agent->generic_map_store["power_curves"]->size();
          conf.scale.min = -100;
          conf.scale.max = 0;
          conf.tooltip.show = true;
          conf.tooltip.format = "x=%.2f, y=%.2f";
          conf.grid_x.show = false;
          conf.grid_y.show = true;
          conf.grid_y.size = 10.0f;
          conf.grid_y.subticks = 5;
          conf.frame_size = ImVec2(wMax.x - wMin.x, wMax.y - wMin.y);
          conf.line_thickness = 2.f;
          ImGui::Plot("plot", conf);
          ImGui::End();
        }
      }

      for (auto agent : simulations[current_simulation].agents) {
        if (agent->generic_store.find("antenna") !=
            agent->generic_store.end()) {
          ImGui::Begin(("Antenna of agent " + agent->get_type() + " " +
                        std::to_string(agent->get_id()))
                           .c_str());
          ImVec2 wMin = ImGui::GetWindowContentRegionMin();
          ImVec2 wMax = ImGui::GetWindowContentRegionMax();
          wMin.x += ImGui::GetWindowPos().x;
          wMin.y += ImGui::GetWindowPos().y;
          wMax.x += ImGui::GetWindowPos().x;
          wMax.y += ImGui::GetWindowPos().y;

          for (auto new_agent : simulations[current_simulation].agents) {
            if (agent->get_id() != new_agent->get_id()) {
              glm::dvec3 rtp = angle(agent, new_agent);
              int index = (((rtp[2] + M_PI) * 180.0) / M_PI) *
                          (wMax.x - wMin.x) / 360.0f;
              ImGui::GetWindowDrawList()->AddLine(
                  ImVec2(wMin.x + index, wMin.y),
                  ImVec2(wMin.x + index, wMax.y),
                  ImGui::GetColorU32(ImVec4(1.0, 0.0, 0.0, 1.0)), 6.0f);
            }
          }

          uint32_t an = 100;
          ImGui::PlotConfig conf;
          conf.values.count = 360;
          conf.values.ys = std::get<std::vector<float>>(
                               agent->generic_store["antenna"]->values)
                               .data();
          conf.values.count = std::get<std::vector<float>>(
                                  agent->generic_store["antenna"]->values)
                                  .size();
          conf.scale.min = -100;
          conf.scale.max = 0;
          conf.tooltip.show = true;
          conf.tooltip.format = "x=%.2f, y=%.2f";
          conf.grid_x.show = false;
          conf.grid_y.show = true;
          conf.grid_y.size = 10.0f;
          conf.grid_y.subticks = 5;
          conf.frame_size = ImVec2(wMax.x - wMin.x, wMax.y - wMin.y);
          conf.line_thickness = 2.f;
          ImGui::Plot("plot", conf);
          ImGui::End();
        }
      }
    }

    ImGui::Begin("Robot Visualization");

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glViewport(0, 0, width, height);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    //
    // GRID
    //
    glUseProgram(gridShaderProgram);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glEnableVertexAttribArray(gridAttr);
    glVertexAttribPointer(gridAttr, 4, GL_FLOAT, GL_FALSE, 0, (void *)0);

    int gridProjectionLocation =
        glGetUniformLocation(gridShaderProgram, "projection");
    int gridModelLocation = glGetUniformLocation(gridShaderProgram, "model");
    int gridViewLocation = glGetUniformLocation(gridShaderProgram, "view");
    glm::mat4 projection =
        glm::perspective(glm::radians(55.0f), 1.0f, 0.1f, 800.0f) * flip;
    glUniformMatrix4fv(gridProjectionLocation, 1, GL_FALSE,
                       glm::value_ptr(projection));
    glm::mat4 view = glm::lookAt(glm::vec3(1 / zoom, 1 / zoom, 1 / zoom),
                                 glm::vec3(0.0, 0.0, 0.0), glm::vec3(0, 0, 1));
    glUniformMatrix4fv(gridViewLocation, 1, GL_FALSE, glm::value_ptr(view));
    glm::mat4 model = glm::mat4(1.0);
    glUniformMatrix4fv(gridModelLocation, 1, GL_FALSE, glm::value_ptr(model));

    // Floor
    glDrawArrays(GL_LINES, 0, gridVertices.size() / 4);

    // Oxz
    model =
        glm::rotate(model, glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    glUniformMatrix4fv(gridModelLocation, 1, GL_FALSE, glm::value_ptr(model));
    glDrawArrays(GL_LINES, 0, gridVertices.size() / 4);

    // Oyz
    model =
        glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glUniformMatrix4fv(gridModelLocation, 1, GL_FALSE, glm::value_ptr(model));

    glDrawArrays(GL_LINES, 0, gridVertices.size() / 4);

    glDisableVertexAttribArray(gridAttr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    //
    // NODES
    //
    glUseProgram(nodesShaderProgram);
    glLineWidth(5);
    glBindVertexArray(nodesVAO);
    glBindBuffer(GL_ARRAY_BUFFER, nodesVBO);
    glEnableVertexAttribArray(nodesAttr);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * nodes.size(), &nodes.front(),
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(nodesAttr, 4, GL_FLOAT, GL_FALSE, 0, (void *)0);

    int nodesProjectionLocation =
        glGetUniformLocation(nodesShaderProgram, "projection");
    int nodesModelLocation = glGetUniformLocation(nodesShaderProgram, "model");
    int nodesViewLocation = glGetUniformLocation(nodesShaderProgram, "view");
    glUniformMatrix4fv(nodesProjectionLocation, 1, GL_FALSE,
                       glm::value_ptr(projection));
    glUniformMatrix4fv(nodesViewLocation, 1, GL_FALSE, glm::value_ptr(view));
    model = glm::mat4(1.0);
    glUniformMatrix4fv(nodesModelLocation, 1, GL_FALSE, glm::value_ptr(model));

    glDrawArrays(GL_LINES, 0, nodes.size() / 4);

    glDisableVertexAttribArray(nodesAttr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glLineWidth(1);

    glDisable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    ImVec2 vMin = ImGui::GetWindowContentRegionMin();
    ImVec2 vMax = ImGui::GetWindowContentRegionMax();
    ImGui::Image((void *)(intptr_t)texColorBuffer,
                 ImVec2(vMax.x - vMin.x, vMax.y - vMin.y));
    ImGui::End();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }
  // Cleanup
  glDeleteRenderbuffers(1, &rbo);
  glDeleteFramebuffers(1, &framebuffer);
  glDeleteTextures(1, &texColorBuffer);
  glDeleteVertexArrays(1, &gridVAO);
  glDeleteBuffers(1, &gridVBO);
  glDeleteVertexArrays(1, &nodesVAO);
  glDeleteBuffers(1, &nodesVBO);

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
