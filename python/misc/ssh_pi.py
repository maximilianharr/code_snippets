import paramiko

ssh = paramiko.SSHClient()
ssh.connect('192.168.43.248', username='pi', password='konsti')
ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("height = 10")
