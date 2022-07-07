from jinja2 import Environment, FileSystemLoader, select_autoescape
import argparse
import sys
import os
import re
import shutil

def main(args):
    # parsing out the arguments from the script
    snake_name = args.package_name
    msg_type = args.msg_type
    msg_topic = args.msg_topic
    snake_to_camel_re = re.compile(r'([A-Za-z0-9]+)')
    camel_name = ''.join([l[0].upper() + l[1:] for l in snake_to_camel_re.findall(snake_name)])
    # rendering the template
    env = Environment(
        loader=FileSystemLoader("./node_template"),
        autoescape=select_autoescape()
    )
    basedir = f"../src/{snake_name}/"
    temp_names = {
        f"{basedir}{snake_name}/{snake_name}.py": "node_template/node_template.py",
        f"{basedir}setup.cfg": "setup.cfg",
        f"{basedir}setup.py": "setup.py",
        f"{basedir}package.xml": "package.xml"
        }
    os.mkdir(basedir)
    os.mkdir(basedir+snake_name)

    for outname, temp_name in temp_names.items():
        template = env.get_template(temp_name)
        with open(outname, 'w+') as outfile:   
            outfile.write(template.render(
                    camel_name=camel_name,
                    snake_name=snake_name,
                    msg_type=msg_type,
                    msg_topic=msg_topic
                  ))
    dest = basedir
    shutil.copytree("node_template/resource", f"{dest}/resource")
    shutil.copytree("node_template/test",f"{dest}/test") 

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Creates sensor package.')
    parser.add_argument('package_name', type=str, help='name of package in snake case')
    parser.add_argument('--msg_type', required=False, default="TODO", type=str, nargs=1, help='message type for sensor node')
    parser.add_argument('--msg_topic', required=False, default="TODO", type=str, nargs=1, help='message topic for sensor node')
    args = parser.parse_args(sys.argv[1:])
    main(args)
