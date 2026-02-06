from launch import LaunchDescription  # Launch dosyası için ana sınıf, başlatılacak tüm node ve argümanları listeler
from launch.actions import DeclareLaunchArgument  # Launch argümanı tanımlamak için kullanılır
import os  # Dosya yollarını yönetmek için Python kütüphanesi
from ament_index_python.packages import get_package_share_directory  # ROS2 paketlerinin yolunu almak için
from launch_ros.actions import Node  # ROS2 node'larını başlatmak için Launch aracılığıyla kullanılır
from launch_ros.parameter_descriptions import ParameterValue  # Node parametrelerini tanımlamak için
from launch.substitutions import Command, LaunchConfiguration  # Launch argümanlarını ve komut çıktısını okumak için

def generate_launch_description():  # Launch dosyasının ana fonksiyonu, tüm işlemleri burada tanımlar

    # Launch argümanı: kullanıcı terminalden model path'i verebilir, yoksa default kullanılır
    model_arg = DeclareLaunchArgument( 
        name = "model",  # Argüman adı
        default_value = os.path.join(
            get_package_share_directory("bumperbot_description"),  # Paketin bulunduğu klasör
            "urdf", 
            "bumperbot.xacro"  # Default olarak kullanılacak xacro dosyası
        ),
        description="Absolute path to robot URDF file"  # Kullanıcıya argümanın amacı hakkında bilgi
    )

    # robot_description parametresi: xacro'u URDF'e çevirip Node parametresi olarak kullanır
    robot_description = ParameterValue(
        Command(["xacro", LaunchConfiguration("model")]),  # Terminalde yaptığımız xacro komutunu otomatik çalıştırır
        value_type=str  # Parametre türü string olmalı
    )

    # Robot State Publisher node'u: URDF modelini okuyup ROS topic'ine yayınlar
    robot_state_publisher = Node(
        package="robot_state_publisher",  # Node hangi pakette
        executable="robot_state_publisher",  # Paket içindeki hangi program çalıştırılacak
        parameters=[{"robot_description": robot_description}]  # Node parametreleri, URDF dosyası burada gönderilir
    )

    # Joint State Publisher GUI node'u: robot eklemlerini GUI üzerinden kontrol etmeye yarar
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",  # Node paketi
        executable="joint_state_publisher_gui"  # Node executable
    )

    # RViz node'u: robotu görselleştirmek için
    rviz_node = Node(
        package="rviz2",  # RViz paketi
        executable="rviz2",  # RViz çalıştırılacak
        name="rviz2",  # Node adı
        output="screen",  # Log mesajları terminalde gözüksün
        arguments=["-d", os.path.join(  # RViz açılırken hangi config dosyasını yükleyecek
            get_package_share_directory("bumperbot_description"),  # Paketin yolu
            "rviz", 
            "display.rviz"  # RViz konfigürasyonu
        )]
    )

    # LaunchDescription: launch dosyasına eklenecek tüm işlemler burada listelenir
    return LaunchDescription([
        model_arg,  # Argümanı ekle
        robot_state_publisher,  # Robot State Publisher node'u ekle
        joint_state_publisher,  # Joint State Publisher GUI node'u ekle
        rviz_node  # RViz node'u ekle
    ])