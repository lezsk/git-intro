git的应用（windows系统）

安装git，在某个目录下右键打开git bash终端。

一、建立本地仓库和提交

在gitee、github或者其他托管网站上复制目标仓库的地址

以下都会以git-intro为例说明

在终端中输入 

```
`git clone https://github.com/lezsk/git-intro.git`
 cd git-intro
```

就进入了仓库,看一下目前有的目录

```
$ ls
README.md  source/
```

假设在git-intro目录下添加一个git-intro.md文件，得到

```
$ ls
git-intro.md  README.md  source/
```

用`git status`查看更改信息，系统会自动列出推荐的操作

```
$ git status
On branch main
Your branch is up to date with 'origin/main'./*这是我们的远程分支目录*/

Untracked files:
  (use "git add <file>..." to include in what will be committed)
        git-intro.md

nothing added to commit but untracked files present (use "git add" to track)

```

可以使用`git add git-intro.md`或者`git add .`或者`git add -A`来添加本目录下untracked file

```
$ git add .
$ git status
On branch main
Your branch is up to date with 'origin/main'.

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
        new file:   git-intro.md
```

此时我们的本地仓库中的/.../git-intro/git-intro.md从unstaged变成staged状态（可以在gitkraken右边栏看到此状态）

然后用git commit命令进行本地提交，附加更新说明文字。

```
$ git commit -m "add git-intro.md"
[main 4cd223b] add git-intro.md
 1 file changed, 29 insertions(+)
 create mode 100644 git-intro.md

```

最后git push命令同步到github

此时，如果另一个同事对某个文件做了更改