import rospy

rospy.init_node('meu_drone_node')

rospy.loginfo("O nó do drone foi iniciado com sucesso!")

# Mantém o nó rodando até você apertar Ctrl+C
rospy.spin()