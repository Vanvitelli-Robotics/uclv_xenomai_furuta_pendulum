#include <QApplication>
#include <QPushButton>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QScreen>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QProcess>
#include <QGroupBox>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <xenopkg_interfaces/srv/joint_srv.hpp>
#include <xenopkg_interfaces/srv/set_mode.hpp>
#include <xenopkg_interfaces/msg/float32_header.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <std_msgs/msg/string.hpp>

float theta = 1.0;
float omega = 1.0;
std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> setserial;
std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> setlqr;
std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> setswingup;
std::shared_ptr<rclcpp::Client<xenopkg_interfaces::srv::SetMode>> setmode;
std::shared_ptr<rclcpp::Client<xenopkg_interfaces::srv::JointSrv>> movejoint;
QTextEdit *logTextEdit;
QProcess *process = new QProcess();
void topic_callback(const xenopkg_interfaces::msg::Float32Header::SharedPtr message)
{
  theta = message->data[0];
  omega = message->data[1];
}

// void //log_message(const std::string &msg)
// {
//     // Converte il messaggio C++ std::string in QString per Qt
//     QString logMsg = QString::fromStdString(msg);

//     // Aggiungi il messaggio al QTextEdit
//     logTextEdit->append(logMsg);
// }

void swing_up_on()
{
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " activating swingup... ");
    // log_message(" disactivating lqr...");

    auto setswingup_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    setswingup_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setswingup->async_send_request(setswingup_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,swingup activate");
    // log_message(" request completed ,lqr disactivate");
    //  check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }
}

void swing_up_off()
{
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " disactivating swingup... ");
    // log_message(" disactivating lqr...");

    auto setswingup_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    setswingup_request->data = false;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setswingup->async_send_request(setswingup_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,swingup disactivate");
    // log_message(" request completed ,lqr disactivate");
    //  check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }
}

void startcb()
{
  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");

  setserial->wait_for_service();
  setlqr->wait_for_service();
  setmode->wait_for_service();
  movejoint->wait_for_service();
  RCLCPP_INFO(node->get_logger(), "Servers UP");
  // log_message("Servers UP");
  sleep(5);
  // porto il robot alla posizione desiderata qf
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " moving to qf... ");
    // log_message("moving to qf... ");
    auto movejoint_request = std::make_shared<xenopkg_interfaces::srv::JointSrv::Request>();
    movejoint_request->qf = {-0.75, 1.3962634016, -1.3962634016, 0.0, 0.0, M_PI_2};

    // chiamo il servizio e aspetto la risposta
    auto future_result = movejoint->async_send_request(movejoint_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,robot in qf");
    // log_message(" request completed ,robot in qf");
    //  check del successo del servizio
    if (!future_result.get()->completed)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }

  // attivo la seriale aspettando che il pendolo sia fermo
  sleep(2);
  {

    RCLCPP_INFO_STREAM(node->get_logger(), "encoder activating...");
    // log_message("encoder activating...");
    auto set_serial_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    set_serial_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }

    // Variabili per la gestione del tempo

    rclcpp::Time timeout(5, 0);
    rclcpp::Time last_zero_time = node->now();
    // Attendere il primo messaggio
    RCLCPP_INFO(node->get_logger(), "calibration...");
    // log_message( "calibration...");
    bool velwaszero = 0;

    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
      rate.sleep();
      // rclcpp::wait_for_message<xenopkg_interfaces::msg::Float32Header>(msg, node, "/encoder_data", std::chrono::milliseconds(1));
      rclcpp::spin_some(node);
      // printf("omega:%f",omega);
      if (velwaszero)
      {
        if (fabs(omega) > 1e-3)
        {
          velwaszero = 0;
          continue;
        }
        if ((node->now().seconds() - last_zero_time.seconds()) > timeout.seconds())
        {
          break;
        }
      }
      else
      {

        if (fabs(omega) < 1e-3)
        {
          // Se il valore cambia da 0, resetta il timer
          velwaszero = 1;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " timer reset");
          // log_message( " timer reset");
          last_zero_time = node->now(); // Reset
        }
      }
    }

    // adesso devo resettare lo zero dell'encoder

    set_serial_request->data = false;

    // chiamo il servizio e aspetto la risposta
    auto future_result1 = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result1) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result1.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }

    // rimando un true per far partire il publish dei valori corretti
    set_serial_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result2 = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result2) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result2.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }

  // adesso metto il robot il velocity mode
  sleep(2);
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " setting velocity mode... ");
    // log_message(" setting velocity mode...");
    auto setmode_request = std::make_shared<xenopkg_interfaces::srv::SetMode::Request>();
    setmode_request->mode = 2;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setmode->async_send_request(setmode_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,velocity mode");
    // log_message(" request completed ,velocity mode");
    //  check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }

  // attivo lqr
  sleep(2);
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " activating lqr... ");
    // log_message(" activating lqr...");

    auto setlqr_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    setlqr_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setlqr->async_send_request(setlqr_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,lqr activate");
    // log_message("request completed ,lqr activate");

    // check del successo del servizio
    // if (!future_result.get()->success)
    // {
    //   RCLCPP_ERROR_STREAM(node->get_logger(),
    //                       "ERROR, SERVICE NOT SUCCESSFUL... message: ");
    //   return ;
    // }
  }
}

void stopcb()
{

  // Impostazione modalità a 'false' (presumibilmente "ferma" il robot)
  {
    auto setmode_request = std::make_shared<xenopkg_interfaces::srv::SetMode::Request>();
    setmode_request->mode = 1; // Modalità di posizione, per fermare il controllo
    auto future_result = setmode->async_send_request(setmode_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "Error to set position mode");
      // log_message(" Error to set position mode");
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Position mode");
      // log_message(" Position mode");
    }
  }

  {
    RCLCPP_INFO_STREAM(node->get_logger(), " disactivating lqr... ");
    // log_message(" disactivating lqr...");

    auto setlqr_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    setlqr_request->data = false;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setlqr->async_send_request(setlqr_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,lqr disactivate");
    // log_message(" request completed ,lqr disactivate");
    //  check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }

  {

    RCLCPP_INFO_STREAM(node->get_logger(), "encoder disactivating...");
    // log_message(" encoder disactivating...");

    auto set_serial_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    set_serial_request->data = false;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      // log_message("ERROR, SERVICE RESULT NOT AVAILABLE...");
      return;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      // log_message(" ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return;
    }
  }

  RCLCPP_INFO_STREAM(node->get_logger(), " end ");
}


void start_driver() {
 
    system("gnome-terminal -- sudo -E -S ~/prova_xenomai_ws/run_driver.sh");
  
}
void start_network() {
 
    system("gnome-terminal -- ros2 launch xenopkg kalmancontroller.launch.py");
 
}


int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("pendulum_interface");

  setserial = node->create_client<std_srvs::srv::SetBool>("set_serial_srv");
  setlqr = node->create_client<std_srvs::srv::SetBool>("activate_lqr_srv");
  setmode = node->create_client<xenopkg_interfaces::srv::SetMode>("set_mode_srv");
  movejoint = node->create_client<xenopkg_interfaces::srv::JointSrv>("joint_position_srv");
  setswingup = node->create_client<std_srvs::srv::SetBool>("set_swingup_srv");
  auto subscription = node->create_subscription<xenopkg_interfaces::msg::Float32Header>("/encoder_data", 1, topic_callback);

  QApplication app(argc, argv);

  // Creazione della finestra principale
QWidget window;
window.setWindowTitle("Controllo del Pendolo");
window.setStyleSheet("background-color: #f0f0f0;");
window.resize(600, 400);

// Layout principale
QVBoxLayout layout(&window);
layout.setAlignment(Qt::AlignTop);
layout.setSpacing(20);
layout.setContentsMargins(20, 20, 20, 20);

// Etichetta di titolo
// QLabel label("Furuta Pendulum", &window);
// QFont font("Arial", 24, QFont::Bold);
// label.setFont(font);
// label.setAlignment(Qt::AlignCenter);
// layout.addWidget(&label);

// Sezione Driver
QGroupBox driverGroup("Driver MECA", &window);
QVBoxLayout driverLayout;
QLabel startDriverDesc("1) Avvia il driver MECA per il controllo del pendolo.");
startDriverDesc.setStyleSheet("color: #555;");
driverLayout.addWidget(&startDriverDesc);

QPushButton startDriverButton("Start MECA Driver");
startDriverButton.setStyleSheet("background-color: #4CAF50; color: white; font-size: 18px; padding: 10px;");
driverLayout.addWidget(&startDriverButton);
driverGroup.setLayout(&driverLayout);
layout.addWidget(&driverGroup);

// Sezione Rete ROS2
QGroupBox networkGroup("Rete ROS2", &window);
QVBoxLayout networkLayout;
QLabel startNetworkDesc("2) Avvia la rete ROS2 per la comunicazione tra i nodi.");
startNetworkDesc.setStyleSheet("color: #555;");
networkLayout.addWidget(&startNetworkDesc);

QPushButton startNetwork("Start ROS2 Network");
startNetwork.setStyleSheet("background-color: #008CBA; color: white; font-size: 18px; padding: 10px;");
networkLayout.addWidget(&startNetwork);
networkGroup.setLayout(&networkLayout);
layout.addWidget(&networkGroup);

// Sezione Pendolo
QGroupBox pendulumGroup("Pendolo", &window);
QVBoxLayout pendulumLayout;

QLabel startPendulumDesc("3) Avvia il pendolo.");
startPendulumDesc.setStyleSheet("color: #555;");
pendulumLayout.addWidget(&startPendulumDesc);
QPushButton startButton("Start Pendulum");
startButton.setStyleSheet("background-color: #4CAF50; color: white; font-size: 18px; padding: 10px;");
startButton.setIcon(QIcon::fromTheme("system-run"));
startButton.setIconSize(QSize(24, 24));
pendulumLayout.addWidget(&startButton);

QLabel stopPendulumDesc("Ferma il pendolo.");
stopPendulumDesc.setStyleSheet("color: #555;");
pendulumLayout.addWidget(&stopPendulumDesc);
QPushButton stopButton("Stop Pendulum");
stopButton.setStyleSheet("background-color: #f44336; color: white; font-size: 18px; padding: 10px;");
stopButton.setIcon(QIcon::fromTheme("process-stop"));
stopButton.setIconSize(QSize(24, 24));
pendulumLayout.addWidget(&stopButton);
pendulumGroup.setLayout(&pendulumLayout);
layout.addWidget(&pendulumGroup);

// Sezione Swing Up
QGroupBox swingGroup("Modalità Swing Up", &window);
QVBoxLayout swingLayout;
QLabel swingUpOnDesc("Attiva la modalità Swing Up.");
swingUpOnDesc.setStyleSheet("color: #555;");
swingLayout.addWidget(&swingUpOnDesc);
QPushButton swingUpOnButton("Swing Up ON");
swingUpOnButton.setStyleSheet("background-color: #008CBA; color: white; font-size: 18px; padding: 10px;");
swingLayout.addWidget(&swingUpOnButton);

QLabel swingUpOffDesc("Disattiva la modalità Swing Up.");
swingUpOffDesc.setStyleSheet("color: #555;");
swingLayout.addWidget(&swingUpOffDesc);
QPushButton swingUpOffButton("Swing Up OFF");
swingUpOffButton.setStyleSheet("background-color: #f44336; color: white; font-size: 18px; padding: 10px;");
swingLayout.addWidget(&swingUpOffButton);
swingGroup.setLayout(&swingLayout);
layout.addWidget(&swingGroup);

    
  // logTextEdit = new QTextEdit(&window);
  // logTextEdit->setReadOnly(true);
  // logTextEdit->setStyleSheet("background-color: #e8e8e8; font-size: 14px;");
  // layout.addWidget(logTextEdit);

  // Connessione del pulsante Start per eseguire tutte le operazioni
  QObject::connect(&startButton, &QPushButton::clicked, startcb);

  // Connessione del pulsante Stop per fermare tutte le operazioni
  QObject::connect(&stopButton, &QPushButton::clicked, stopcb);

  QObject::connect(&swingUpOnButton, &QPushButton::clicked, swing_up_on);
  QObject::connect(&swingUpOffButton, &QPushButton::clicked, swing_up_off);
  
  QObject::connect(&startDriverButton, &QPushButton::clicked, start_driver);

  QObject::connect(&startNetwork, &QPushButton::clicked, start_network);


  // Assegna il layout alla finestra principale
  window.setLayout(&layout);
  window.show();

  // Ciclo dell'applicazione Qt
  int result = app.exec();
  delete logTextEdit;
  delete process;
  // Shutdown di ROS2 prima di uscire
  rclcpp::shutdown();

  return result;
}
