from setuptools import find_packages, setup

setup(
    name='navegacao_autonoma',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/navegacao_autonoma']),
        ('share/navegacao_autonoma', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel.chiovato',
    maintainer_email='samuel.chiovato@sou.inteli.edu.br',
    description='Navegacao com mapa completo e exploracao por sensores.',
    license='MIT',
    entry_points={'console_scripts': ['resolver = navegacao_autonoma.node:main']},
)
